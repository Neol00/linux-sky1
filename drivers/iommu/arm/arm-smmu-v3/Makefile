# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_ARM_SMMU_V3) += arm_smmu_v3.o
arm_smmu_v3-y := arm-smmu-v3.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_IOMMUFD) += arm-smmu-v3-iommufd.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_WALK) += arm-smmu-v3-walk.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_DEBUG) += arm-smmu-v3-debug.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_DUMP) += arm-smmu-v3-dump.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_SVA) += arm-smmu-v3-sva.o
arm_smmu_v3-$(CONFIG_TEGRA241_CMDQV) += tegra241-cmdqv.o

obj-$(CONFIG_ARM_SMMU_V3_KUNIT_TEST) += arm-smmu-v3-test.o
