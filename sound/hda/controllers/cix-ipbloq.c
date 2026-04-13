// SPDX-License-Identifier: GPL-2.0
// Copyright 2024 Cix Technology Group Co., Ltd.

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/string.h>

#include <sound/hda_codec.h>
#include "hda_controller.h"

#define CIX_IPBLOQ_ADDR_HOST_TO_HDAC_OFFSET	0x90000000

/*
 * SMC SCMI transport for audio power domain control.
 *
 * On CIX SKY1 the SCP firmware exposes the power domain protocol (0x11) on
 * the SMC transport channel, NOT on the mailbox channel.  In the DT path the
 * generic power-domain framework (genpd) handles this transparently via
 * power-domains = <&smc_devpd SKY1_PD_AUDIO>, but in the ACPI path no SCMI
 * POWER_STATE_SET is ever issued — the ACPI PRSS method is routed through the
 * mailbox and silently succeeds without actually toggling power.
 *
 * The audio clock driver (clk-sky1-audss) already powers the domain on during
 * its own probe, but PM genpd tears it down again ("Disabling unused power
 * domains") before the HDA controller has a chance to probe.  We therefore
 * must explicitly power the domain on ourselves, matching what the Mali GPU
 * driver does for its own domain.
 *
 * Shared-memory layout and func-id match the DT arm,scmi-smc transport.
 */
#define SMC_SCMI_FUNC_ID	0xc2000001UL
#define SMC_SCMI_SHMEM_PHYS	0x84380000UL
#define SMC_SCMI_SHMEM_SIZE	0x80

#define SHMEM_OFF_CHAN_STATUS	0x04
#define SHMEM_OFF_FLAGS		0x10
#define SHMEM_OFF_LENGTH	0x14
#define SHMEM_OFF_MSG_HEADER	0x18
#define SHMEM_OFF_MSG_PAYLOAD	0x1c
#define SHMEM_CHAN_FREE		BIT(0)

#define SCMI_HDR(proto, msg)	\
	(((msg) & 0xFF) | (((proto) & 0xFF) << 10))

#define SKY1_PD_AUDIO		0
#define SCMI_PD_STATE_ON	0
#define SCMI_PD_STATE_OFF	BIT(30)

/*
 * ACPI-mode hardware init for HDA codec power-on.
 *
 * In ACPI boot the firmware does not define the fch_gpio3 controller or
 * apply HDA pin-mux, so the codec PDN# (power-down-bar) is never driven
 * and the HDA link pins may not be muxed.  We fix this with direct MMIO.
 */

/* Pinctrl – IOMUX base for pad configuration (function + config) */
#define SKY1_PINCTRL_BASE	0x04170000UL
#define SKY1_PINCTRL_SIZE	0x200
#define SKY1_PIN_SIZE		4	/* one 32-bit register per pin */

/* HDA pin assignments: pin_number, register_value (mux=0 | config) */
struct sky1_hda_pin_cfg {
	unsigned int pin;
	u32 val;
};

/*
 * Function 0 = HDA for pins 42-48.
 * Config: bits[8:7]=mux(0), bit6=PU, bit5=PD, bit4=ST, bits[3:0]=DS
 *   PULL_DOWN | ST | DS_LEVEL12 = 0x3c
 *   PULL_UP   | ST | DS_LEVEL12 = 0x5c
 */
static const struct sky1_hda_pin_cfg sky1_hda_pins[] = {
	{ 42, 0x3c },	/* HDA_BITCLK: PD|ST|DS12 */
	{ 43, 0x3c },	/* HDA_RST_L:  PD|ST|DS12 */
	{ 44, 0x3c },	/* HDA_SDIN0:  PD|ST|DS12 */
	{ 45, 0x5c },	/* HDA_SDOUT0: PU|ST|DS12 */
	{ 46, 0x5c },	/* HDA_SYNC:   PU|ST|DS12 */
	{ 47, 0x3c },	/* HDA_SDIN1:  PD|ST|DS12 */
	{ 48, 0x3c },	/* HDA_SDOUT1: PD|ST|DS12 */
};

/* fch_gpio3 – Cadence GPIO controller for codec PDN# */
#define FCH_GPIO3_BASE		0x04150000UL
#define FCH_GPIO3_SIZE		0x30
#define CDNS_GPIO_BYPASS_MODE	0x00
#define CDNS_GPIO_DIRECTION	0x04
#define CDNS_GPIO_OUTPUT_EN	0x08
#define CDNS_GPIO_OUTPUT_VALUE	0x0c
#define FCH_GPIO3_PDB_PIN	5	/* codec power-down-bar on gpio3 pin 5 */

/* FCH reset controller – deassert GPIO APB reset */
#define FCH_SRC_BASE		0x04160000UL
#define FCH_SRC_SIZE		0x90
#define FCH_SW_RST_BUS_OFF	0x0c
#define FCH_GPIO_RST_BIT	BIT(21)

/* SCMI clock protocol for GPIO APB clock */
#define SCMI_PROTO_CLOCK	0x14
#define SCMI_CLK_CONFIG_SET	0x07
#define CLK_FCH_GPIO_APB	262	/* CLK_TREE_FCH_GPIO_APB */

static int cix_smc_scmi_send(struct device *dev, u32 protocol, u32 msg_id,
			     const u32 *payload, unsigned int payload_words,
			     u32 *resp_status)
{
	void __iomem *shmem;
	struct arm_smccc_res res;
	int timeout;
	unsigned int i;

	shmem = ioremap(SMC_SCMI_SHMEM_PHYS, SMC_SCMI_SHMEM_SIZE);
	if (!shmem) {
		dev_err(dev, "cannot map SCMI SMC shmem\n");
		return -ENOMEM;
	}

	timeout = 1000;
	while (timeout-- > 0) {
		if (ioread32(shmem + SHMEM_OFF_CHAN_STATUS) & SHMEM_CHAN_FREE)
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		dev_err(dev, "SCMI SMC channel busy\n");
		iounmap(shmem);
		return -EBUSY;
	}

	iowrite32(0, shmem + SHMEM_OFF_CHAN_STATUS);
	iowrite32(0, shmem + SHMEM_OFF_FLAGS);
	iowrite32(4 + payload_words * 4, shmem + SHMEM_OFF_LENGTH);
	iowrite32(SCMI_HDR(protocol, msg_id), shmem + SHMEM_OFF_MSG_HEADER);
	for (i = 0; i < payload_words; i++)
		iowrite32(payload[i], shmem + SHMEM_OFF_MSG_PAYLOAD + i * 4);

	arm_smccc_smc(SMC_SCMI_FUNC_ID,
		      SMC_SCMI_SHMEM_PHYS >> 12, 0,
		      0, 0, 0, 0, 0, &res);

	if (res.a0) {
		dev_err(dev, "SMC SCMI call failed: a0=0x%lx\n", res.a0);
		iowrite32(SHMEM_CHAN_FREE, shmem + SHMEM_OFF_CHAN_STATUS);
		iounmap(shmem);
		return -EIO;
	}

	if (resp_status)
		*resp_status = ioread32(shmem + SHMEM_OFF_MSG_PAYLOAD);

	iowrite32(SHMEM_CHAN_FREE, shmem + SHMEM_OFF_CHAN_STATUS);
	iounmap(shmem);
	return 0;
}

static int cix_audio_power_set(struct device *dev, u32 state)
{
	u32 payload[3] = { 0, SKY1_PD_AUDIO, state };
	u32 status;
	int ret;

	ret = cix_smc_scmi_send(dev, 0x11, 0x04, payload, 3, &status);
	if (ret)
		return ret;
	if (status != 0) {
		dev_err(dev, "SCMI POWER_STATE_SET(audio, 0x%x): error 0x%x\n",
			state, status);
		return -EIO;
	}
	return 0;
}

static int cix_audio_power_on(struct device *dev)
{
	int ret;

	ret = cix_audio_power_set(dev, SCMI_PD_STATE_ON);
	if (!ret)
		dev_info(dev, "audio power domain ON via SMC SCMI\n");
	return ret;
}

static void cix_audio_power_off(struct device *dev)
{
	int ret;

	ret = cix_audio_power_set(dev, SCMI_PD_STATE_OFF);
	if (!ret)
		dev_dbg(dev, "audio power domain OFF via SMC SCMI\n");
	else
		dev_warn(dev, "failed to power off audio domain: %d\n", ret);
}

/*
 * cix_acpi_hda_hw_init - Configure pin-mux and codec GPIO for ACPI boot.
 *
 * In ACPI mode the DSDT does not define the fch_gpio3 controller, so the
 * ALC256 codec PDN# pin is never driven HIGH (codec stays powered off).
 * The HDA pad mux may also not be applied.  We configure everything here
 * via direct MMIO before the HDA link reset attempts codec enumeration.
 */
static int cix_acpi_hda_hw_init(struct device *dev)
{
	void __iomem *pinctrl, *gpio, *rst;
	u32 val;
	int i, ret;

	/* 1. Enable GPIO APB clock via SCMI */
	{
		u32 clk_payload[2] = { CLK_FCH_GPIO_APB, 1 };
		u32 status;

		ret = cix_smc_scmi_send(dev, SCMI_PROTO_CLOCK,
					SCMI_CLK_CONFIG_SET,
					clk_payload, 2, &status);
		if (ret || status)
			dev_warn(dev, "SCMI clock enable GPIO APB: ret=%d status=0x%x (continuing)\n",
				 ret, status);
		else
			dev_info(dev, "GPIO APB clock enabled via SCMI\n");
	}

	/* 2. Deassert GPIO APB reset */
	rst = ioremap(FCH_SRC_BASE, FCH_SRC_SIZE);
	if (!rst) {
		dev_warn(dev, "cannot map FCH reset controller\n");
	} else {
		val = readl(rst + FCH_SW_RST_BUS_OFF);
		if (!(val & FCH_GPIO_RST_BIT)) {
			val |= FCH_GPIO_RST_BIT;
			writel(val, rst + FCH_SW_RST_BUS_OFF);
			usleep_range(100, 200);
			dev_info(dev, "GPIO APB reset deasserted\n");
		} else {
			dev_info(dev, "GPIO APB reset already deasserted\n");
		}
		iounmap(rst);
	}

	/* 3. Configure HDA pin mux */
	pinctrl = ioremap(SKY1_PINCTRL_BASE, SKY1_PINCTRL_SIZE);
	if (!pinctrl) {
		dev_warn(dev, "cannot map pinctrl for HDA mux\n");
	} else {
		for (i = 0; i < ARRAY_SIZE(sky1_hda_pins); i++) {
			void __iomem *reg = pinctrl +
				sky1_hda_pins[i].pin * SKY1_PIN_SIZE;
			u32 old = readl(reg);

			writel(sky1_hda_pins[i].val, reg);
			dev_info(dev, "pin %u mux: 0x%03x -> 0x%03x\n",
				 sky1_hda_pins[i].pin, old,
				 sky1_hda_pins[i].val);
		}
		iounmap(pinctrl);
	}

	/* 4. Drive codec PDN# (power-down-bar) HIGH via fch_gpio3 pin 5 */
	gpio = ioremap(FCH_GPIO3_BASE, FCH_GPIO3_SIZE);
	if (!gpio) {
		dev_err(dev, "cannot map fch_gpio3 – codec will stay off!\n");
		return -ENOMEM;
	}

	/* Disable bypass for pin 5 (use GPIO mode) */
	val = readl(gpio + CDNS_GPIO_BYPASS_MODE);
	val &= ~BIT(FCH_GPIO3_PDB_PIN);
	writel(val, gpio + CDNS_GPIO_BYPASS_MODE);

	/* Set pin 5 as output (direction = 0 means output) */
	val = readl(gpio + CDNS_GPIO_DIRECTION);
	val &= ~BIT(FCH_GPIO3_PDB_PIN);
	writel(val, gpio + CDNS_GPIO_DIRECTION);

	/* Enable output for pin 5 */
	val = readl(gpio + CDNS_GPIO_OUTPUT_EN);
	val |= BIT(FCH_GPIO3_PDB_PIN);
	writel(val, gpio + CDNS_GPIO_OUTPUT_EN);

	/* Drive pin 5 HIGH */
	val = readl(gpio + CDNS_GPIO_OUTPUT_VALUE);
	val |= BIT(FCH_GPIO3_PDB_PIN);
	writel(val, gpio + CDNS_GPIO_OUTPUT_VALUE);

	iounmap(gpio);
	dev_info(dev, "codec PDN# (fch_gpio3 pin %d) driven HIGH\n",
		 FCH_GPIO3_PDB_PIN);

	/* 5. Wait for codec to power up (ALC256 needs ~20ms after PDN# HIGH) */
	msleep(50);

	return 0;
}

#define CIX_IPBLOQ_JACKPOLL_DEFAULT_TIME_MS	1000
#define CIX_IPBLOQ_POWER_SAVE_DEFAULT_TIME_MS	100

struct cix_ipbloq_hda {
	struct azx chip;
	struct device *dev;
	void __iomem *regs;

	struct reset_control_bulk_data resets[1];
	struct clk_bulk_data clocks[2];
	unsigned int nresets;
	unsigned int nclocks;

	struct work_struct probe_work;

	struct gpio_desc *pdb_gpiod;
	struct gpio_desc *depop_mute_gpiod;

	const char *sname;

	bool acpi_power_on; /* audio domain powered via SMC SCMI */
};

static const struct hda_controller_ops cix_ipbloq_hda_ops;

/* alc256 cix evb init verb table */
static unsigned int alc256_cix_evb_init_verbs[] = {
	/* Realtek High Definition Audio Configuration - Version : 5.0.3.3
	 * Realtek HD Audio Codec : ALC256
	 * PCI PnP ID : PCI\VEN_8086&DEV_2668&SUBSYS_129E10EC
	 * HDA Codec PnP ID : HDAUDIO\FUNC_01&VEN_10EC&DEV_0256&SUBSYS_10EC129E
	 * The number of verb command block : 16
	 *
	 * NID 0x12 : 0x90A60130
	 * NID 0x13 : 0x40000000
	 * NID 0x14 : 0x90170110
	 * NID 0x18 : 0x411111F0
	 * NID 0x19 : 0x04A11040
	 * NID 0x1A : 0x411111F0
	 * NID 0x1B : 0x411111F0
	 * NID 0x1D : 0x4068996D
	 * NID 0x1E : 0x411111F0
	 * NID 0x21 : 0x04211020
	 */

	/* ==== HDA Codec Subsystem ID Verb-table ===== */
	/* HDA Codec Subsystem ID  : 0x10EC129E */
	0x0017209E,
	0x00172112,
	0x001722EC,
	0x00172310,

	/* ==== Pin Widget Verb-table ===== */
	/* Widget node 0x01 */
	0x0017FF00,
	0x0017FF00,
	0x0017FF00,
	0x0017FF00,
	/* 1bit reset */
	0x0205001A,
	0x0204C00B,
	0x0205001A,
	0x0204800B,
	/* Pin widget 0x12 - DMIC */
	0x01271C30,
	0x01271D01,
	0x01271EA6,
	0x01271F90,
	/* Pin widget 0x13 - DMIC */
	0x01371C00,
	0x01371D00,
	0x01371E00,
	0x01371F40,
	/* Pin widget 0x14 - Front (Port-D) */
	0x01471C10,
	0x01471D01,
	0x01471E17,
	0x01471F90,
	/* Pin widget 0x18 - NPC */
	0x01871CF0,
	0x01871D11,
	0x01871E11,
	0x01871F41,
	/* Pin widget 0x19 - MIC2 (Port-F) */
	0x01971C40,
	0x01971D10,
	0x01971EA1,
	0x01971F04,
	/* Pin widget 0x1A - LINE1 (Port-C) */
	0x01A71CF0,
	0x01A71D11,
	0x01A71E11,
	0x01A71F41,
	/* Pin widget 0x1B - LINE2 (Port-E) */
	0x01B71CF0,
	0x01B71D11,
	0x01B71E11,
	0x01B71F41,
	/* Pin widget 0x1D - BEEP-IN */
	0x01D71C6D,
	0x01D71D99,
	0x01D71E68,
	0x01D71F40,
	/* Pin widget 0x1E - S/PDIF-OUT */
	0x01E71CF0,
	0x01E71D11,
	0x01E71E11,
	0x01E71F41,
	/* Pin widget 0x21 - HP1-OUT (Port-I) */
	0x02171C20,
	0x02171D10,
	0x02171E21,
	0x02171F04,

	0x02050010,
	0x02040020,
	0x02050038,
	0x02046981,

	0x02050008,
	0x02046A6C,
	0x0205001B,
	0x02040A4B,

	0x0205003C,
	0x02040354,
	0x0205003C,
	0x02040314,

	0x02050046,
	0x02040004,
	0x05750003,
	0x057409A3,
};

/* alc256 cix orion o6 init verb table */
static unsigned int alc256_cix_orion_o6_init_verbs[] = {
	/* Realtek High Definition Audio Configuration - Version : 5.0.3.3
	 * Realtek HD Audio Codec : ALC256
	 * PCI PnP ID : PCI\VEN_8086&DEV_2668&SUBSYS_129E10EC
	 * HDA Codec PnP ID : HDAUDIO\FUNC_01&VEN_10EC&DEV_0256&SUBSYS_10EC129E
	 * The number of verb command block : 16
	 *
	 * NID 0x12 : 0x40000000
	 * NID 0x13 : 0x411111F0
	 * NID 0x14 : 0x90170110
	 * NID 0x18 : 0x411111F0
	 * NID 0x19 : 0x01A11030
	 * NID 0x1A : 0x02A19040
	 * NID 0x1B : 0x02014020
	 * NID 0x1D : 0x4045C069
	 * NID 0x1E : 0x411111F0
	 * NID 0x21 : 0x0121101F
	 */

	/* ==== HDA Codec Subsystem ID Verb-table ===== */
	/* HDA Codec Subsystem ID  : 0x10EC129E */
	0x0017209E,
	0x00172112,
	0x001722EC,
	0x00172310,

	/* ==== Pin Widget Verb-table ===== */
	/* Widget node 0x01 */
	0x0017FF00,
	0x0017FF00,
	0x0017FF00,
	0x0017FF00,
	/* 1bit reset */
	0x0205001A,
	0x0204C00B,
	0x0205001A,
	0x0204800B,
	/* Pin widget 0x12 - DMIC */
	0x01271C00,
	0x01271D00,
	0x01271E00,
	0x01271F40,
	/* Pin widget 0x13 - DMIC */
	0x01371CF0,
	0x01371D11,
	0x01371E11,
	0x01371F41,
	/* Pin widget 0x14 - Front (Port-D) */
	0x01471C10,
	0x01471D01,
	0x01471E17,
	0x01471F90,
	/* Pin widget 0x18 - NPC */
	0x01871CF0,
	0x01871D11,
	0x01871E11,
	0x01871F41,
	/* Pin widget 0x19 - MIC2 (Port-F) */
	0x01971C30,
	0x01971D10,
	0x01971EA1,
	0x01971F01,
	/* Pin widget 0x1A - LINE1 (Port-C) */
	0x01A71C40,
	0x01A71D90,
	0x01A71EA1,
	0x01A71F02,
	/* Pin widget 0x1B - LINE2 (Port-E) */
	0x01B71C20,
	0x01B71D40,
	0x01B71E01,
	0x01B71F02,
	/* Pin widget 0x1D - BEEP-IN */
	0x01D71C69,
	0x01D71DC0,
	0x01D71E45,
	0x01D71F40,
	/* Pin widget 0x1E - S/PDIF-OUT */
	0x01E71CF0,
	0x01E71D11,
	0x01E71E11,
	0x01E71F41,
	/* Pin widget 0x21 - HP1-OUT (Port-I) */
	0x02171C1F,
	0x02171D10,
	0x02171E21,
	0x02171F01,

	0x02050010,
	0x02040020,
	0x02050038,
	0x02046981,

	0x02050008,
	0x02046A4C,
	0x0205001B,
	0x02040A4B,

	0x0205003C,
	0x02040354,
	0x0205003C,
	0x02040314,

	0x02050046,
	0x02040004,
	0x05750003,
	0x057409A3,
};

/* alc269 cix orapi 6p init verb table */
static unsigned int alc269_cix_orapi_6p_init_verbs[] = {
	/* Realtek High Definition Audio Configuration - Version : 5.0.3.3
	 * Realtek HD Audio Codec : ALC269-VC3
	 * PCI PnP ID : PCI\VEN_8086&DEV_2668&SUBSYS_129E10EC
	 * HDA Codec PnP ID : HDAUDIO\FUNC_01&VEN_10EC&DEV_0269&SUBSYS_10EC129E
	 * The number of verb command block : 17
	 *
	 * NID 0x12 : 0x40000000
	 * NID 0x14 : 0x90170110
	 * NID 0x15 : 0x0421101F
	 * NID 0x17 : 0x411111F0
	 * NID 0x18 : 0x04A11020
	 * NID 0x19 : 0x90A7012F
	 * NID 0x1A : 0x411111F0
	 * NID 0x1B : 0x411111F0
	 * NID 0x1D : 0x40538205
	 * NID 0x1E : 0x411111F0
	 * NID 0x20 : 0x0000FFFF
	 */

	/* ==== HDA Codec Subsystem ID Verb-table ===== */
	/* HDA Codec Subsystem ID  : 0x10EC129E */
	0x0017209E,
	0x00172112,
	0x001722EC,
	0x00172310,

	/* ==== Pin Widget Verb-table ===== */
	/* Widget node 0x01 */
	0x0017FF00,
	0x0017FF00,
	0x0017FF00,
	0x0017FF00,
	/* Pin widget 0x12 - DMIC */
	0x01271C00,
	0x01271D00,
	0x01271E00,
	0x01271F40,
	/* Pin widget 0x14 - SPEAKER-OUT (Port-D) */
	0x01471C10,
	0x01471D01,
	0x01471E17,
	0x01471F90,
	/* Pin widget 0x15 - HP-OUT (Port-A) */
	0x01571C1F,
	0x01571D10,
	0x01571E21,
	0x01571F04,
	/* Pin widget 0x17 - MONO-OUT (Port-H) */
	0x01771CF0,
	0x01771D11,
	0x01771E11,
	0x01771F41,
	/* Pin widget 0x18 - MIC1 (Port-B) */
	0x01871C20,
	0x01871D10,
	0x01871EA1,
	0x01871F04,
	/* Pin widget 0x19 - MIC2 (Port-F) */
	0x01971C2F,
	0x01971D01,
	0x01971EA7,
	0x01971F90,
	/* Pin widget 0x1A - LINE1 (Port-C) */
	0x01A71CF0,
	0x01A71D11,
	0x01A71E11,
	0x01A71F41,
	/* Pin widget 0x1B - LINE2 (Port-E) */
	0x01B71CF0,
	0x01B71D11,
	0x01B71E11,
	0x01B71F41,
	/* Pin widget 0x1D - PC-BEEP */
	0x01D71C05,
	0x01D71D82,
	0x01D71E53,
	0x01D71F40,
	/* Pin widget 0x1E - S/PDIF-OUT */
	0x01E71CF0,
	0x01E71D11,
	0x01E71E11,
	0x01E71F41,
	/* Widget node 0x20 */
	0x02050018,
	0x02040184,
	0x0205001C,
	0x02040800,
	/* Widget node 0x20 - 1 */
	0x02050024,
	0x02040000,
	0x02050004,
	0x02040080,
	/* Widget node 0x20 - 2 */
	0x02050008,
	0x02040300,
	0x0205000C,
	0x02043F00,
	/* Widget node 0x20 - 3 */
	0x02050015,
	0x02048002,
	0x02050015,
	0x02048002,
	/* Widget node 0x0C */
	0x00C37080,
	0x00270610,
	0x00D37080,
	0x00370610,
};

static int cix_ipbloq_hda_config_init_verbs(struct hdac_bus *bus, unsigned int vendor_id)
{
	struct snd_card *card = dev_get_drvdata(bus->dev);
	struct azx *chip = card->private_data;
	struct cix_ipbloq_hda *hda = container_of(chip, struct cix_ipbloq_hda, chip);
	unsigned int *init_verbs = NULL;
	unsigned int size = 0;
	int i;

	dev_info(bus->dev, "config_init_verbs: vendor_id=0x%08x sname=%s\n",
		 vendor_id, hda->sname ? hda->sname : "(null)");

	switch (vendor_id) {
	case 0x10ec0256:
		if (hda->sname && !strcmp(hda->sname, "CIX SKY1 EVB HDA")) {
			dev_info(bus->dev, "using CIX SKY1 EVB HDA verbs\n");
			init_verbs = alc256_cix_evb_init_verbs;
			size = ARRAY_SIZE(alc256_cix_evb_init_verbs);
		} else if (hda->sname && !strcmp(hda->sname, "CIX SKY1 ORION O6 HDA")) {
			dev_info(bus->dev, "using CIX SKY1 ORION O6 HDA verbs\n");
			init_verbs = alc256_cix_orion_o6_init_verbs;
			size = ARRAY_SIZE(alc256_cix_orion_o6_init_verbs);
		} else {
			/*
			 * In ACPI mode the _DSD may not set "cix,model", so
			 * sname defaults to "cix-ipbloq-hda".  Fall back to
			 * the Orion O6 verb table as that is the primary board
			 * using this SoC with ALC256.
			 */
			dev_info(bus->dev,
				 "sname '%s' unknown for ALC256, defaulting to Orion O6 verbs\n",
				 hda->sname ? hda->sname : "(null)");
			init_verbs = alc256_cix_orion_o6_init_verbs;
			size = ARRAY_SIZE(alc256_cix_orion_o6_init_verbs);
		}
		break;
	case 0x10ec0269:
		if (hda->sname && !strcmp(hda->sname, "CIX SKY1 ORAPI 6P HDA")) {
			dev_info(bus->dev, "using CIX SKY1 ORAPI 6P HDA verbs\n");
			init_verbs = alc269_cix_orapi_6p_init_verbs;
			size = ARRAY_SIZE(alc269_cix_orapi_6p_init_verbs);
		} else {
			dev_warn(bus->dev,
				 "no init verbs for ALC269 on board '%s'\n",
				 hda->sname ? hda->sname : "(null)");
			return 0;
		}
		break;
	default:
		dev_warn(bus->dev,
			 "no init verbs for codec vendor 0x%08x\n", vendor_id);
		return 0;
	}

	if (!init_verbs || !size)
		return 0;

	for (i = 0; i < size; i++)
		bus->ops->command(bus, init_verbs[i]);

	dev_info(bus->dev, "sent %u init verbs for codec 0x%08x\n",
		 size, vendor_id);
	return 0;
}

static int cix_ipbloq_hda_dev_disconnect(struct snd_device *device)
{
	struct azx *chip = device->device_data;

	chip->bus.shutdown = 1;

	return 0;
}

static int cix_ipbloq_hda_dev_free(struct snd_device *device)
{
	struct azx *chip = device->device_data;
	struct cix_ipbloq_hda *hda = container_of(chip, struct cix_ipbloq_hda, chip);

	cancel_work_sync(&hda->probe_work);

	if (azx_bus(chip)->chip_init) {
		azx_stop_all_streams(chip);
		azx_stop_chip(chip);
	}

	azx_free_stream_pages(chip);
	azx_free_streams(chip);
	snd_hdac_bus_exit(azx_bus(chip));

	return 0;
}

static int cix_ipbloq_hda_init_chip(struct azx *chip, struct platform_device *pdev)
{
	struct cix_ipbloq_hda *hda = container_of(chip, struct cix_ipbloq_hda, chip);
	struct hdac_bus *bus = azx_bus(chip);
	struct resource *res;

	hda->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hda->regs)) {
		dev_err(&pdev->dev, "failed to get and ioremap resource\n");
		return PTR_ERR(hda->regs);
	}

	bus->remap_addr = hda->regs;
	bus->addr = res->start;

	return 0;
}

static int cix_ipbloq_hda_init(struct azx *chip, struct platform_device *pdev)
{
	struct cix_ipbloq_hda *hda = container_of(chip, struct cix_ipbloq_hda, chip);
	struct hdac_bus *bus = azx_bus(chip);
	struct snd_card *card = chip->card;
	const char *sname = NULL, *drv_name = "cix-ipbloq-hda";
	unsigned short gcap;
	int irq_id, err;

	err = cix_ipbloq_hda_init_chip(chip, pdev);
	if (err)
		return err;

	irq_id = platform_get_irq(pdev, 0);
	if (irq_id < 0) {
		dev_err(&pdev->dev, "failed to get the irq\n");
		return irq_id;
	}

	err = devm_request_irq(chip->card->dev, irq_id, azx_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, chip);
	if (err) {
		dev_err(chip->card->dev,
			"Unable to request IRQ %d, disabling device\n",
			irq_id);
		return err;
	}

	bus->irq = irq_id;
	bus->dma_stop_delay = 100;
	card->sync_irq = bus->irq;

	gcap = azx_readw(chip, GCAP);
	dev_info(card->dev, "chipset global capabilities = 0x%x\n", gcap);

	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;

	/* initialize streams */
	err = azx_init_streams(chip);
	if (err < 0) {
		dev_err(card->dev, "failed to initialize streams: %d\n", err);
		return err;
	}

	err = azx_alloc_stream_pages(chip);
	if (err < 0) {
		dev_err(card->dev, "failed to allocate stream pages: %d\n", err);
		return err;
	}

	/* initialize chip */
	azx_init_chip(chip, 1);

	/* codec detection - retry with increasing delays for SoC platforms
	 * where the codec needs extra time after link reset to enumerate.
	 * The core snd_hdac_bus_reset_link() only waits 1ms after CRST
	 * deassertion, which is insufficient for some codecs (e.g. ALC256)
	 * on non-PCI HDA controllers where BCLK may take longer to stabilize.
	 */
	if (!bus->codec_mask) {
		int retry;

		for (retry = 0; retry < 50; retry++) {
			msleep(10);
			bus->codec_mask = snd_hdac_chip_readw(bus, STATESTS);
			if (bus->codec_mask)
				break;
		}
		if (!bus->codec_mask) {
			dev_err(card->dev, "no codecs found after %dms\n",
				(retry + 1) * 10);
			return -ENODEV;
		}
		dev_info(card->dev, "codec detected after %dms retry\n",
			 (retry + 1) * 10);
	}
	dev_info(card->dev, "codec detection mask = 0x%lx\n", bus->codec_mask);

	/* driver name */
	strscpy(card->driver, drv_name, sizeof(card->driver));

	/* shortname for card */
	device_property_read_string(&pdev->dev, "cix,model", &sname);
	if (!sname)
		sname = drv_name;
	/* use to distinguish boards later when select verb table */
	hda->sname = sname;

	if (strlen(sname) > sizeof(card->shortname))
		dev_info(card->dev, "truncating shortname for card\n");
	strscpy(card->shortname, sname, sizeof(card->shortname));

	/* longname for card */
	snprintf(card->longname, sizeof(card->longname),
		 "%s at 0x%lx irq %i",
		 card->shortname, bus->addr, bus->irq);

	return 0;
}

static void cix_ipbloq_hda_probe_work(struct work_struct *work)
{
	struct cix_ipbloq_hda *hda = container_of(work, struct cix_ipbloq_hda, probe_work);
	struct platform_device *pdev = to_platform_device(hda->dev);
	struct azx *chip = &hda->chip;
	struct hdac_bus *bus = azx_bus(chip);
	int err;

	pm_runtime_get_sync(hda->dev);

	to_hda_bus(bus)->bus_probing = 1;

	err = cix_ipbloq_hda_init(chip, pdev);
	if (err < 0)
		goto out_free;

	/* create codec instances */
	err = azx_probe_codecs(chip, 8);
	if (err < 0)
		goto out_free;

	/*
	 * Send board-specific init verbs (pin configs, subsystem ID, etc.)
	 * BEFORE codec driver binding.  The config_init_verbs callback was
	 * registered but the HDA core never invokes it, so call it here
	 * for each discovered codec.  This must happen after CORB/RIRB are
	 * running (azx_init_chip in cix_ipbloq_hda_init) and after the
	 * codec device is created (azx_probe_codecs), but before the codec
	 * driver reads pin configurations (azx_codec_configure).
	 */
	if (bus->config_init_verbs) {
		struct hda_codec *codec;

		list_for_each_codec(codec, &chip->bus) {
			dev_info(hda->dev,
				 "sending init verbs for codec vendor 0x%08x\n",
				 codec->core.vendor_id);
			bus->config_init_verbs(bus, codec->core.vendor_id);
		}
	}

	/* Pre-load the Realtek codec module before configure tries to bind */
	request_module("snd-hda-codec-alc269");

	err = azx_codec_configure(chip);
	if (err < 0)
		goto out_free;

	err = snd_card_register(chip->card);
	if (err < 0)
		goto out_free;

	chip->running = 1;

	to_hda_bus(bus)->bus_probing = 0;

	snd_hda_set_power_save(&chip->bus, CIX_IPBLOQ_POWER_SAVE_DEFAULT_TIME_MS);

	dev_info(hda->dev, "cix ipbloq hda probed\n");

 out_free:
	pm_runtime_put(hda->dev);
	return; /* no error return from async probe */
}

static int cix_ipbloq_hda_create(struct snd_card *card,
				 unsigned int driver_caps,
				 struct cix_ipbloq_hda *hda)
{
	static const struct snd_device_ops ops = {
		.dev_disconnect = cix_ipbloq_hda_dev_disconnect,
		.dev_free = cix_ipbloq_hda_dev_free,
	};
	struct azx *chip;
	int err;

	chip = &hda->chip;

	mutex_init(&chip->open_mutex);
	chip->card = card;
	chip->ops = &cix_ipbloq_hda_ops;
	chip->driver_caps = driver_caps;
	chip->driver_type = driver_caps & 0xff;
	chip->dev_index = 0;
	chip->single_cmd = 0;
	chip->codec_probe_mask = -1;
	chip->align_buffer_size = 1;
	chip->jackpoll_interval = msecs_to_jiffies(CIX_IPBLOQ_JACKPOLL_DEFAULT_TIME_MS);
	INIT_LIST_HEAD(&chip->pcm_list);

	/* HD-audio controllers appear pretty inaccurate about the update-IRQ timing.
	 * The IRQ is issued before actually the data is processed. So use stream
	 * link position by default instead of dma position buffer.
	 */
	chip->get_position[0] = chip->get_position[1] = azx_get_pos_lpib;

	INIT_WORK(&hda->probe_work, cix_ipbloq_hda_probe_work);

	err = azx_bus_init(chip, NULL);
	if (err < 0) {
		dev_err(hda->dev, "failed to init bus, err = %d\n", err);
		return err;
	}

	/* RIRBSTS.RINTFL cannot be cleared, cause interrupt storm */
	chip->bus.core.polling_mode = 1;
	chip->bus.core.not_use_interrupts = 1;

	chip->bus.core.aligned_mmio = 1;
	chip->bus.jackpoll_in_suspend = 1;

	/* host and hdac has different memory view (7.0: callback replaced by offset) */
	chip->bus.core.addr_offset = -(dma_addr_t)CIX_IPBLOQ_ADDR_HOST_TO_HDAC_OFFSET;

	/* config init verbs TODO: config from BIOS
	 */
	chip->bus.core.config_init_verbs = cix_ipbloq_hda_config_init_verbs;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0) {
		dev_err(card->dev, "failed to create device, err = %d\n", err);
		return err;
	}

	return 0;
}

static int cix_ipbloq_hda_probe(struct platform_device *pdev)
{
	const unsigned int driver_flags = AZX_DCAPS_PM_RUNTIME;
	struct snd_card *card;
	struct azx *chip;
	struct cix_ipbloq_hda *hda;
	int err;

	hda = devm_kzalloc(&pdev->dev, sizeof(*hda), GFP_KERNEL);
	if (!hda) {
		dev_err(&pdev->dev, "failed to allocate memory for hda\n");
		return -ENOMEM;
	}
	hda->dev = &pdev->dev;
	chip = &hda->chip;

	err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   THIS_MODULE, 0, &card);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to crate card, err = %d\n", err);
		return err;
	}

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (!pdev->dev.dma_mem)  {
		/*
		 * if dev.dma_mem not allcated
		 * we should try to get it from dts
		 */
		err = of_reserved_mem_device_init(&pdev->dev);
		if (err && err != -ENODEV) {
			dev_err(&pdev->dev,
				"failed to init reserved mem for DMA, err = %d\n", err);
			goto out_free;
		}
	}

	hda->resets[hda->nresets++].id = "hda";
	err = devm_reset_control_bulk_get_exclusive(&pdev->dev, hda->nresets,
						    hda->resets);
	if (err) {
		dev_err(&pdev->dev, "failed to get reset, err = %d\n", err);
		goto out_free;
	}

	hda->clocks[hda->nclocks++].id = "sysclk";
	hda->clocks[hda->nclocks++].id = "clk48m";
	err = devm_clk_bulk_get(&pdev->dev, hda->nclocks, hda->clocks);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get clk, err = %d\n", err);
		goto out_free;
	}

	hda->pdb_gpiod = devm_gpiod_get_optional(&pdev->dev, "pdb", GPIOD_OUT_HIGH);
	if (IS_ERR(hda->pdb_gpiod)) {
		err = PTR_ERR(hda->pdb_gpiod);
		dev_err(&pdev->dev, "failed to get pdb gpio, err: %d\n", err);
		goto out_free;
	}
	msleep(20);

	hda->depop_mute_gpiod = devm_gpiod_get_optional(&pdev->dev, "depop-mute", GPIOD_OUT_HIGH);
	if (IS_ERR(hda->depop_mute_gpiod)) {
		err = PTR_ERR(hda->depop_mute_gpiod);
		dev_err(&pdev->dev, "failed to get depop gpio, err: %d\n", err);
		goto out_free;
	}
	gpiod_set_value_cansleep(hda->depop_mute_gpiod, 1);

	err = cix_ipbloq_hda_create(card, driver_flags, hda);
	if (err < 0)
		goto out_free;
	card->private_data = chip;

	dev_set_drvdata(&pdev->dev, card);

	/*
	 * In ACPI mode the audio power domain (SKY1_PD_AUDIO) is not
	 * managed by genpd.  The clock driver powers it on during its
	 * own probe, but genpd tears it back down before we get here.
	 * Power the domain on now so the codec is alive when we try to
	 * enumerate it in probe_work → runtime_resume → azx_init_chip.
	 */
	if (ACPI_COMPANION(&pdev->dev)) {
		err = cix_audio_power_on(&pdev->dev);
		if (err)
			dev_warn(&pdev->dev,
				 "SCMI audio power-on failed (%d), codec may not enumerate\n",
				 err);
		else
			hda->acpi_power_on = true;
		/* Let SCP firmware process the power state change */
		usleep_range(1000, 2000);

		/*
		 * Configure HDA pin mux & drive codec PDN# HIGH.
		 * This must happen after the audio power domain is on.
		 */
		err = cix_acpi_hda_hw_init(&pdev->dev);
		if (err)
			dev_warn(&pdev->dev,
				 "ACPI HW init failed (%d), codec may not enumerate\n",
				 err);
	}

	/* Do NOT call pm_runtime_set_active — let probe_work's
	 * pm_runtime_get_sync trigger runtime_resume which does
	 * clk_enable + reset, matching the 6.6 kernel flow.
	 */
	pm_runtime_enable(hda->dev);
	if (!azx_has_pm_runtime(chip))
		pm_runtime_forbid(hda->dev);

	schedule_work(&hda->probe_work);

	return 0;

out_free:
	snd_card_free(card);
	return err;
}

static void cix_ipbloq_hda_remove(struct platform_device *pdev)
{
	snd_card_free(dev_get_drvdata(&pdev->dev));

	pm_runtime_disable(&pdev->dev);
}

static void cix_ipbloq_hda_shutdown(struct platform_device *pdev)
{
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct azx *chip;

	if (!card)
		return;

	chip = card->private_data;
	if (chip && chip->running)
		azx_stop_chip(chip);
}

static int __maybe_unused cix_ipbloq_hda_suspend(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip;
	struct cix_ipbloq_hda *hda;
	int rc;

	if (!card || !card->private_data)
		return 0;

	chip = card->private_data;
	hda = container_of(chip, struct cix_ipbloq_hda, chip);

	rc = pm_runtime_force_suspend(dev);
	if (rc < 0)
		return rc;
	snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);

	if (hda->depop_mute_gpiod)
		gpiod_set_value_cansleep(hda->depop_mute_gpiod, 0);

	if (hda->pdb_gpiod)
		gpiod_set_value_cansleep(hda->pdb_gpiod, 0);

	return 0;
}

static int __maybe_unused cix_ipbloq_hda_resume(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip;
	struct cix_ipbloq_hda *hda;
	int rc;

	if (!card || !card->private_data)
		return 0;

	chip = card->private_data;
	hda = container_of(chip, struct cix_ipbloq_hda, chip);

	if (hda->pdb_gpiod)
		gpiod_set_value_cansleep(hda->pdb_gpiod, 1);
	msleep(20);

	if (hda->depop_mute_gpiod)
		gpiod_set_value_cansleep(hda->depop_mute_gpiod, 1);

	rc = pm_runtime_force_resume(dev);
	if (rc < 0)
		return rc;
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	return 0;
}

static int __maybe_unused cix_ipbloq_hda_runtime_suspend(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip;
	struct cix_ipbloq_hda *hda;

	if (!card || !card->private_data)
		return 0;

	chip = card->private_data;
	hda = container_of(chip, struct cix_ipbloq_hda, chip);

	dev_dbg(dev, "%s\n", __func__);

	if (chip->running) {
		azx_stop_chip(chip);
		azx_enter_link_reset(chip);
	}

	clk_bulk_disable_unprepare(hda->nclocks, hda->clocks);

	if (hda->acpi_power_on)
		cix_audio_power_off(dev);

	return 0;
}

static int __maybe_unused cix_ipbloq_hda_runtime_resume(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip;
	struct cix_ipbloq_hda *hda;
	int rc;

	if (!card || !card->private_data)
		return -ENODEV;

	chip = card->private_data;
	hda = container_of(chip, struct cix_ipbloq_hda, chip);

	dev_dbg(dev, "%s\n", __func__);

	/* Re-enable audio power domain before touching clocks/resets */
	if (hda->acpi_power_on) {
		rc = cix_audio_power_on(dev);
		if (rc) {
			dev_err(dev, "SCMI audio power-on failed on resume: %d\n", rc);
			return rc;
		}
		usleep_range(1000, 2000);

		/* Re-init pin mux and codec PDN# after power domain comes back */
		cix_acpi_hda_hw_init(dev);
	}

	rc = clk_bulk_prepare_enable(hda->nclocks, hda->clocks);
	if (rc) {
		dev_err(dev, "failed to enable clk bulk, rc: %d\n", rc);
		return rc;
	}

	rc = reset_control_bulk_assert(hda->nresets, hda->resets);
	if (rc) {
		dev_err(dev, "failed to assert reset bulk, rc: %d\n", rc);
		return rc;
	}

	usleep_range(10, 20);

	rc = reset_control_bulk_deassert(hda->nresets, hda->resets);
	if (rc) {
		dev_err(dev, "failed to deassert reset bulk, rc: %d\n", rc);
		return rc;
	}

	/* Allow HDA controller to stabilize and start generating BCLK
	 * after SoC-level reset deassert, before any register access
	 * or GCTL link reset is attempted.
	 */
	usleep_range(1000, 1500);

	if (chip && chip->running)
		azx_init_chip(chip, 1);

	return 0;
}

static const struct dev_pm_ops cix_ipbloq_hda_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(cix_ipbloq_hda_suspend,
				cix_ipbloq_hda_resume)
	SET_RUNTIME_PM_OPS(cix_ipbloq_hda_runtime_suspend,
			   cix_ipbloq_hda_runtime_resume, NULL)
};

static const struct of_device_id cix_ipbloq_hda_match[] = {
	{ .compatible = "cix,sky1-ipbloq-hda", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, cix_ipbloq_hda_match);

static const struct acpi_device_id cix_ipbloq_hda_acpi_match[] = {
	{ "CIXH6020" },
	{},
};
MODULE_DEVICE_TABLE(acpi, cix_ipbloq_hda_acpi_match);

static struct platform_driver cix_ipbloq_hda_driver = {
	.driver = {
		.name = "cix-ipbloq-hda",
		.pm = &cix_ipbloq_hda_pm,
		.of_match_table = cix_ipbloq_hda_match,
		.acpi_match_table = cix_ipbloq_hda_acpi_match,
	},
	.probe = cix_ipbloq_hda_probe,
	.remove = cix_ipbloq_hda_remove,
	.shutdown = cix_ipbloq_hda_shutdown,
};
module_platform_driver(cix_ipbloq_hda_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CIX IPBLOQ HDA bus driver");
MODULE_AUTHOR("Joakim Zhang <joakim.zhang@cixtech.com>");
