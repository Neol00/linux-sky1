// SPDX-License-Identifier: GPL-2.0
/*
 * CIX Sky1 Audio Subsystem CRU syscon driver
 *
 * Provides a regmap for the audio CRU (Clock Resource Unit) at 0x07110000.
 * Required so that fw_devlink can resolve the "audss_cru" supplier
 * dependency for the audss-clk and audss-reset drivers.
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

static const struct regmap_config cix_audss_cru_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xFFFC,
};

static int cix_audss_cru_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;
	struct regmap *regmap;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource\n");
		return -ENODEV;
	}

	/*
	 * Use devm_ioremap (not devm_ioremap_resource) because multiple
	 * devices (ACRU, DCRU, CRU0) may share the same physical address
	 * and devm_ioremap_resource would fail on the second one.
	 */
	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "failed to ioremap %pR\n", res);
		return -ENOMEM;
	}

	regmap = devm_regmap_init_mmio(&pdev->dev, base,
				       &cix_audss_cru_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&pdev->dev, "failed to init regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	dev_info(&pdev->dev, "registered regmap at %pR\n", res);
	return 0;
}

static const struct acpi_device_id cix_audss_cru_acpi_match[] = {
	{ "CIXHA018", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, cix_audss_cru_acpi_match);

static struct platform_driver cix_audss_cru_driver = {
	.probe = cix_audss_cru_probe,
	.driver = {
		.name = "cix-audss-cru",
		.acpi_match_table = ACPI_PTR(cix_audss_cru_acpi_match),
	},
};
module_platform_driver(cix_audss_cru_driver);

MODULE_DESCRIPTION("CIX Sky1 Audio Subsystem CRU syscon driver");
MODULE_LICENSE("GPL");
