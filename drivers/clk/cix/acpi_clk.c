// SPDX-License-Identifier: GPL-2.0
/*
 *Copyright 2024 Cix Technology Group Co., Ltd.
 */
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/acpi.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include "acpi_clk.h"

#define GET_CLOCK_RATE		0x00000001
#define SET_CLOCK_RATE		0X00000002
#define SET_CLOCK_CONFIG	0X00000003

#define CLOCK_REVISION_ID	1
#define CLOCK_ENABLE		BIT(0)
#define CLOCK_DISABLE		0
#define CLK_MASK		(0xffffffff)
#define SUCCESS		0

/*
 * CIX Sky1 SCMI-over-SMC clock rate set.
 *
 * The SCP firmware has two SCMI transport channels:
 *   - Mailbox (0x065d0000): used by ACPI SClK/GClK methods
 *   - SMC (shmem 0x84380000, func 0xc2000001): used by DT for power/clock
 *
 * The mailbox transport silently accepts CLOCK_RATE_SET but never actually
 * reprograms certain PLLs (audio, display).  The SMC transport reaches the
 * correct SCP agent.
 */
#define CIX_SMC_FUNC_ID		0xc2000001UL
#define CIX_SMC_SHMEM_PHYS	0x84380000UL
#define CIX_SMC_SHMEM_SIZE	0x80

#define CIX_SH_CHAN_STATUS	0x04
#define CIX_SH_FLAGS		0x10
#define CIX_SH_LENGTH		0x14
#define CIX_SH_MSG_HEADER	0x18
#define CIX_SH_MSG_PAYLOAD	0x1c
#define CIX_SH_CHAN_FREE	BIT(0)

#define CIX_SCMI_HDR(proto, msg) \
	(((msg) & 0xFF) | (((proto) & 0xFF) << 10))

static DEFINE_MUTEX(acpi_smc_scmi_lock);

static int acpi_cix_smc_clock_rate_set(struct device *dev,
				       u32 clk_id, u64 rate)
{
	void __iomem *shmem;
	struct arm_smccc_res res;
	u32 status;
	int timeout;

	shmem = ioremap(CIX_SMC_SHMEM_PHYS, CIX_SMC_SHMEM_SIZE);
	if (!shmem)
		return -ENOMEM;

	mutex_lock(&acpi_smc_scmi_lock);

	timeout = 1000;
	while (timeout-- > 0) {
		if (ioread32(shmem + CIX_SH_CHAN_STATUS) & CIX_SH_CHAN_FREE)
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		mutex_unlock(&acpi_smc_scmi_lock);
		iounmap(shmem);
		return -EBUSY;
	}

	iowrite32(0, shmem + CIX_SH_CHAN_STATUS);
	iowrite32(0, shmem + CIX_SH_FLAGS);
	/* Length = header(4) + payload(16): flags + id + rate_lo + rate_hi */
	iowrite32(20, shmem + CIX_SH_LENGTH);
	/* protocol 0x14 = CLOCK, message 0x05 = CLOCK_RATE_SET */
	iowrite32(CIX_SCMI_HDR(0x14, 0x05), shmem + CIX_SH_MSG_HEADER);
	iowrite32(0, shmem + CIX_SH_MSG_PAYLOAD);
	iowrite32(clk_id, shmem + CIX_SH_MSG_PAYLOAD + 4);
	iowrite32((u32)(rate & 0xFFFFFFFF), shmem + CIX_SH_MSG_PAYLOAD + 8);
	iowrite32((u32)(rate >> 32), shmem + CIX_SH_MSG_PAYLOAD + 12);

	arm_smccc_smc(CIX_SMC_FUNC_ID,
		      CIX_SMC_SHMEM_PHYS >> 12, 0,
		      0, 0, 0, 0, 0, &res);

	status = ioread32(shmem + CIX_SH_MSG_PAYLOAD);
	iowrite32(CIX_SH_CHAN_FREE, shmem + CIX_SH_CHAN_STATUS);

	mutex_unlock(&acpi_smc_scmi_lock);
	iounmap(shmem);

	if (res.a0) {
		dev_warn(dev, "SMC SCMI transport error: a0=0x%lx\n", res.a0);
		return -EIO;
	}
	if (status != 0) {
		dev_warn(dev, "SMC CLOCK_RATE_SET(clk %u, %llu Hz): err 0x%x\n",
			 clk_id, rate, status);
		return -EIO;
	}
	return 0;
}

static u64 acpi_cix_smc_clock_rate_get(struct device *dev, u32 clk_id)
{
	void __iomem *shmem;
	struct arm_smccc_res res;
	u32 status, rate_lo, rate_hi;
	int timeout;

	shmem = ioremap(CIX_SMC_SHMEM_PHYS, CIX_SMC_SHMEM_SIZE);
	if (!shmem)
		return 0;

	mutex_lock(&acpi_smc_scmi_lock);

	timeout = 1000;
	while (timeout-- > 0) {
		if (ioread32(shmem + CIX_SH_CHAN_STATUS) & CIX_SH_CHAN_FREE)
			break;
		udelay(10);
	}
	if (timeout <= 0) {
		mutex_unlock(&acpi_smc_scmi_lock);
		iounmap(shmem);
		return 0;
	}

	iowrite32(0, shmem + CIX_SH_CHAN_STATUS);
	iowrite32(0, shmem + CIX_SH_FLAGS);
	/* Length = header(4) + payload(4): clk_id */
	iowrite32(8, shmem + CIX_SH_LENGTH);
	/* protocol 0x14 = CLOCK, message 0x06 = CLOCK_RATE_GET */
	iowrite32(CIX_SCMI_HDR(0x14, 0x06), shmem + CIX_SH_MSG_HEADER);
	iowrite32(clk_id, shmem + CIX_SH_MSG_PAYLOAD);

	arm_smccc_smc(CIX_SMC_FUNC_ID,
		      CIX_SMC_SHMEM_PHYS >> 12, 0,
		      0, 0, 0, 0, 0, &res);

	status = ioread32(shmem + CIX_SH_MSG_PAYLOAD);
	rate_lo = ioread32(shmem + CIX_SH_MSG_PAYLOAD + 4);
	rate_hi = ioread32(shmem + CIX_SH_MSG_PAYLOAD + 8);
	iowrite32(CIX_SH_CHAN_FREE, shmem + CIX_SH_CHAN_STATUS);

	mutex_unlock(&acpi_smc_scmi_lock);
	iounmap(shmem);

	if (res.a0 || status != 0)
		return 0;

	return ((u64)rate_hi << 32) | rate_lo;
}

static LIST_HEAD(aclk_list);
static LIST_HEAD(aclk_hw_list);

static struct acpi_device *acpi_obj_path_to_adev(union acpi_object *obj)
{
        struct acpi_device *adev = NULL;
        char *path;
        acpi_handle handle;
        acpi_status status;

        if (!obj || !obj->string.length)
                return NULL;

        path = obj->string.pointer;
        status = acpi_get_handle(NULL, path, &handle);
        if (ACPI_FAILURE(status)) {
                return NULL;
        }

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 0, 0)
        adev = acpi_bus_get_acpi_device(handle);
#else
        adev = acpi_fetch_acpi_dev(handle);
#endif

        return adev;
}

static struct acpi_device *acpi_obj_to_adev(union acpi_object *obj)
{
	if (!obj)
		return NULL;

	if (obj->type == ACPI_TYPE_LOCAL_REFERENCE)
		return acpi_fetch_acpi_dev(obj->reference.handle);
	else if (obj->type == ACPI_TYPE_STRING)
		return acpi_obj_path_to_adev(obj);
	else
		return NULL;
}

static int acpi_clock_config_set(struct device *dev, u32 clk_id, u32 config)
{
	acpi_handle handle = ACPI_HANDLE(dev);
	acpi_status status;
	u32 buf_val[1];
	int ret = 0;
	union acpi_object *package;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object args[2] = {
		{ .type = ACPI_TYPE_INTEGER, },
		{ .type = ACPI_TYPE_INTEGER, },
	};

	struct acpi_object_list arg_list = {
		.pointer = args,
		.count = ARRAY_SIZE(args),
	};

	args[0].integer.value = clk_id;
	args[1].integer.value = config;

	status = acpi_evaluate_object(handle, "CLKC", &arg_list, &buffer);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "ACPI evaluation failed\n");
		ret = -ENODEV;
		goto OUT;
	}

	package = buffer.pointer;
	if (!package || package->type != ACPI_TYPE_BUFFER) {
		dev_err(dev, "Couldn't locate correct ACPI buffer\n");
		ret = -ENODEV;
		goto OUT;
	}

	buf_val[0] = *(u32 *)package->buffer.pointer;
	if (buf_val[0] != SUCCESS) {
		dev_err(dev, "ACPI clk[%u] set config[%u] err:%d\n",
					clk_id, config, buf_val[0]);
		ret = -ENODEV;
		goto OUT;
	}

OUT:
	if (buffer.pointer)
		kfree(buffer.pointer);

	return ret;
}

static int acpi_clk_prepare(struct clk_hw *hw)
{
	struct acpi_clk_hw *aclk = to_acpi_clk_hw(hw);

	if (!aclk)
		return -EINVAL;

	return acpi_clock_config_set(aclk->dev, aclk->clk_id, CLOCK_ENABLE);
}

static void acpi_clk_unprepare(struct clk_hw *hw)
{
	struct acpi_clk_hw *aclk = to_acpi_clk_hw(hw);

	if (!aclk)
		return;

	acpi_clock_config_set(aclk->dev, aclk->clk_id, CLOCK_DISABLE);
}

static unsigned long acpi_cix_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct acpi_clk_hw *aclk_hw = to_acpi_clk_hw(hw);
	struct device *dev = aclk_hw->dev;
	u64 rate;

	rate = acpi_cix_smc_clock_rate_get(dev, aclk_hw->clk_id);
	if (rate == 0) {
		dev_warn(dev, "SMC GClK: clk[%u] rate_get failed\n",
			 aclk_hw->clk_id);
		return 0;
	}

	return (unsigned long)rate;
}

static int acpi_cix_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct acpi_clk_hw *aclk_hw = to_acpi_clk_hw(hw);
	struct device *dev = aclk_hw->dev;
	int ret;

	ret = acpi_cix_smc_clock_rate_set(dev, aclk_hw->clk_id, (u64)rate);
	if (ret)
		dev_err(dev, "SMC SClK: clk[%u] set rate %lu failed (%d)\n",
			aclk_hw->clk_id, rate, ret);
	else
		dev_info(dev, "SMC SClK: clk[%u] set rate %lu OK\n",
			 aclk_hw->clk_id, rate);

	return ret;
}

static long cix_acpi_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	/* do not support now */
	return rate;
}

static const struct clk_ops acpi_clk_ops = {
	.prepare = acpi_clk_prepare,
	.unprepare = acpi_clk_unprepare,
	.recalc_rate = acpi_cix_clk_recalc_rate,
	.round_rate = cix_acpi_clk_round_rate,
	.set_rate = acpi_cix_clk_set_rate,
};

static const char *acpi_clk_get_obj_id(union acpi_object *obj)
{
	if (!obj)
		return NULL;

	return obj->string.length ? obj->string.pointer : NULL;
}

static struct clk_hw *acpi_clk_get_hw(unsigned int clk_id)
{
	struct acpi_clk_hw *aclk_hw;

	list_for_each_entry(aclk_hw, &aclk_hw_list, list) {
		if (aclk_hw->clk_id == clk_id)
			return &aclk_hw->hw;
	}

	return NULL;
}

static struct clk_hw *devm_acpi_clk_hw_alloc(struct device *dev,
			unsigned int clk_id)
{
	struct acpi_clk_hw *aclk_hw = NULL;

	aclk_hw = devm_kzalloc(dev, sizeof(*aclk_hw), GFP_KERNEL);
	if (!aclk_hw)
		return ERR_PTR(-ENOMEM);

	aclk_hw->clk_id = clk_id;
	INIT_LIST_HEAD(&aclk_hw->list);
	list_add_tail(&aclk_hw->list, &aclk_hw_list);

	return &aclk_hw->hw;
}

static struct clk_hw *acpi_clk_get_or_create_hw(struct device *dev, int clk_id)
{
	struct clk_init_data init = {};
	struct clk_hw *hw;
	struct acpi_clk_hw *aclk_hw;
	char clk_name[CLK_NAME_LEN];
	int ret;

	hw = acpi_clk_get_hw(clk_id);
	if (!hw) {
		snprintf(clk_name, CLK_NAME_LEN, "ACLK:%04d", clk_id);

		hw = devm_acpi_clk_hw_alloc(dev, clk_id);
		if (!hw)
			goto out;
		hw->init = &init;
		init.name = clk_name;
		init.ops = &acpi_clk_ops;
		init.num_parents = 0;
		init.flags = CLK_GET_RATE_NOCACHE;

		aclk_hw = to_acpi_clk_hw(hw);
		aclk_hw->dev = dev;

		/* register hw clk */
		ret = devm_clk_hw_register(dev, hw);
		if (ret)
			goto out;
	}

	return hw;
out:
	return NULL;
}

int cix_acpi_parse_clkt_handle(acpi_handle handle, const char *name,
			struct clk_hw *(get_hw)(struct device *, int),
			void *data)
{
	struct device *dev = data;
	struct acpi_buffer output = {ACPI_ALLOCATE_BUFFER, NULL};
	union acpi_object *out_obj, *clk_obj, *el[ACLK_MAX];
	acpi_status status;
	int clk_num, pnum, i, ret=0;
	struct acpi_device *adev;
	struct acpi_clk *acpi_clks;
	struct clk_hw *hw;
	const char *con_id, *dname = NULL;
	unsigned int clk_id;

	if (!get_hw)
		return -EINVAL;

	/* Parse the ACPI CLKT table for this CPU. */
	status = acpi_evaluate_object_typed(handle,
			(acpi_string)name, NULL, &output, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		ret = -ENODEV;
		goto out_free;
	}

	out_obj = (union acpi_object *) output.pointer;
	clk_num = out_obj->package.count;
	acpi_clks = devm_kcalloc(dev, clk_num, sizeof(struct acpi_clk), GFP_KERNEL);
	if (!acpi_clks) {
		ret = -ENOMEM;
		goto out_free;
	}

	/* acpi clk register */
	for (i = 0; i < clk_num; i++) {
		clk_obj = &out_obj->package.elements[i];
		pnum = clk_obj->package.count;
		if (pnum < ACLK_DEV)
			continue;

		/* clk package: {id, con_id, [dev_id]} */
		el[0] = &clk_obj->package.elements[0];
		el[1] = &clk_obj->package.elements[1];
		el[2] = pnum > ACLK_DEV ? &clk_obj->package.elements[2] : NULL;

		if ((el[0]->type != ACPI_TYPE_INTEGER)
		    || (el[1]->type != ACPI_TYPE_STRING)
		    || (el[2] && el[2]->type != ACPI_TYPE_LOCAL_REFERENCE
				    && el[2]->type != ACPI_TYPE_STRING))
			continue;

		clk_id = el[0]->integer.value;
		con_id = acpi_clk_get_obj_id(el[1]);
		adev = el[2] ?  acpi_obj_to_adev(el[2]) : NULL;
		dname = adev ? dev_name(&adev->dev) : NULL;

		if (!con_id && !adev)
			continue;

		hw = get_hw(dev, clk_id);
		if (!hw)
			continue;

		acpi_clks[i].hw = hw;
		acpi_clks[i].cl.dev_id = dname;
		acpi_clks[i].cl.con_id = devm_kstrdup(dev, con_id, GFP_KERNEL);
		acpi_clks[i].cl.clk = hw->clk;

		clkdev_add(&acpi_clks[i].cl);
		INIT_LIST_HEAD(&acpi_clks[i].list);
		list_add_tail(&acpi_clks[i].list, &aclk_list);

		dev_dbg(dev, "clk: id[%d] con[%s] dev[%s]\n",
				clk_id, con_id, dname);
	}

out_free:
	if (output.pointer)
		kfree(output.pointer);

	return ret;
}
EXPORT_SYMBOL_GPL(cix_acpi_parse_clkt_handle);

int cix_acpi_parse_clkt(struct device *dev, const char *cname,
		struct clk_hw *(get_hw)(struct device *, int))
{
	if (!dev)
		return -EINVAL;

	return cix_acpi_parse_clkt_handle(ACPI_HANDLE(dev), cname, get_hw, dev);
}
EXPORT_SYMBOL_GPL(cix_acpi_parse_clkt);

static acpi_status acpi_bus_clk_scan(acpi_handle handle, u32 level,
					void *context, void **ret_p)
{
	struct device *dev = context;
	acpi_object_type acpi_type;
	int ret;

	if (ACPI_FAILURE(acpi_get_type(handle, &acpi_type)))
		return AE_OK;

	if (acpi_type != ACPI_TYPE_DEVICE)
		return AE_OK;

	if (!dev)
		return AE_OK;

	ret = cix_acpi_parse_clkt_handle(handle, "CLKT",
					 acpi_clk_get_or_create_hw, dev);

	if (ret && ret != -ENODEV)
		return AE_ERROR;

	return AE_OK;
}

static int cix_acpi_clk_probe(struct platform_device *pdev)
{
	acpi_status status;

	status = acpi_walk_namespace(ACPI_TYPE_ANY, ACPI_ROOT_OBJECT,
				ACPI_UINT32_MAX, acpi_bus_clk_scan,
				NULL, &pdev->dev, NULL);

	/* do not ensure every resource since different configs */
	return ACPI_FAILURE(status) ? -ENODEV : 0;
}

static void cix_acpi_clk_remove(struct platform_device *pdev)
{
}

static const struct acpi_device_id __maybe_unused cix_acpi_clk_match[] = {
	{ "CIXHA010", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, cix_acpi_clk_match);

static struct platform_driver cix_acpi_clk_driver = {
	.driver = {
		.name = "cix_acpi_clk",
		.acpi_match_table = ACPI_PTR(cix_acpi_clk_match),
	},
	.probe = cix_acpi_clk_probe,
	.remove = cix_acpi_clk_remove,
};

static int __init cix_acpi_clk_init(void)
{
	if (acpi_disabled)
		return -ENODEV;

	return platform_driver_register(&cix_acpi_clk_driver);
}
core_initcall(cix_acpi_clk_init);

static void __exit cix_acpi_clk_exit(void)
{
	platform_driver_unregister(&cix_acpi_clk_driver);
}
module_exit(cix_acpi_clk_exit);

MODULE_AUTHOR("Copyright 2024 Cix Technology Group Co., Ltd.");
MODULE_DESCRIPTION("Cix acpi clock driver");
MODULE_LICENSE("GPL v2");
