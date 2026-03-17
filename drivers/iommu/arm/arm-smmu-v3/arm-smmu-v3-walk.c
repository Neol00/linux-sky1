// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 Cix Technology Group Co., Ltd.
 *
 * Author: Zichar Zhang <zichar.zhang@cixtech.com>
 */
#include <linux/iommu.h>
#include <linux/io-pgtable.h>
#include "arm-smmu-v3.h"
#include "arm-smmu-v3-walk.h"

struct smmu_master_info {
	struct arm_smmu_master *master;

	struct list_head list;
};
LIST_HEAD(smmu_master_list);
static struct mutex master_list_lock;

static int io_map_handle(struct pgt_io_map *map, void *data)
{
	struct smmu_master_handle *mhdl = data;

	if (!mhdl || !mhdl->hdl_io_map_s1)
		return -EINVAL;

	return mhdl->hdl_io_map_s1(map->iova,
			map->pa, map->len, map->prot, mhdl->data);
}

static int arm_smmu_pgt_walk(__le64 *cd, struct arm_smmu_domain *smmu_domain,
			struct smmu_master_handle *mhdl)
{
	struct io_pgtable_ops *pgtbl_ops;
	void *pgd;
	__le64 ttbr;

	if (!cd || !smmu_domain || !mhdl)
		return -EINVAL;

	ttbr = le64_to_cpu(cd[1]) & CTXDESC_CD_1_TTB0_MASK;
	pgd = phys_to_virt(ttbr);
	pgtbl_ops = smmu_domain->pgtbl_ops;
	if (!pgtbl_ops)
		return -EINVAL;

	lpae_io_pgt_walk(pgd, pgtbl_ops, 0, io_map_handle, mhdl);

	return 0;
}

static struct arm_smmu_ste *
__arm_smmu_get_step_for_sid(struct arm_smmu_device *smmu, u32 sid)
{
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;

	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB) {
		/* Two-level walk */
		struct arm_smmu_strtab_l2 *l2 =
			cfg->l2.l2ptrs[arm_smmu_strtab_l1_idx(sid)];

		if (!l2)
			return NULL;
		return &l2->stes[arm_smmu_strtab_l2_idx(sid)];
	}

	/* Simple linear lookup */
	return &cfg->linear.table[sid];
}

static int smmu_master_walk_one(struct arm_smmu_master *master,
					struct smmu_master_handle *mhdl)
{
	struct device *mdev, *sdev;
	struct iommu_domain *iommu_domain;
	struct arm_smmu_device *smmu;
	struct arm_smmu_domain *smmu_domain;
	struct iommu_fwspec *fwspec;
	unsigned int sid;
	int ret = 0, i;
	struct arm_smmu_ste *ste;
	struct arm_smmu_cd *cd;

	if (!master || !mhdl || !master->smmu)
		return -EINVAL;

	smmu = master->smmu;
	mdev = master->dev;
	sdev = smmu->dev;

	if (mhdl->smmu_name)
		if (strncmp(dev_name(sdev), mhdl->smmu_name,
					strlen(dev_name(sdev))))
			return -EINVAL;

	if (mhdl->smmu_master_name)
		if (strncmp(dev_name(mdev), mhdl->smmu_master_name,
					strlen(dev_name(mdev))))
			return -EINVAL;

	if (mhdl->init) {
		ret = mhdl->init(master, mhdl->data);
		if (ret < 0)
			return -EINVAL;
	}

	fwspec = dev_iommu_fwspec_get(mdev);
	if (!fwspec)
		return -EINVAL;

	mutex_lock(&smmu->streams_mutex);
	for (i = 0; i < fwspec->num_ids; i++) {
		if (!mhdl->hdl_ste)
			break;

		sid = fwspec->ids[i];
		ste = __arm_smmu_get_step_for_sid(smmu, sid);
		if (!ste)
			continue;

		ret = mhdl->hdl_ste(ste->data, sid, mhdl->data);
		if (ret)
			goto OUT;
	}

	if (!mdev || !mdev->iommu_group)
		goto OUT;

	iommu_domain = iommu_group_default_domain(mdev->iommu_group);
	smmu_domain = to_smmu_domain(iommu_domain);

	/* share the same cd and pgt */
	cd = arm_smmu_get_cd_ptr(master, 0); /* ssid 0 only */
	if (!cd)
		goto OUT;

	if (mhdl->hdl_cd) {
		ret = mhdl->hdl_cd(cd->data, mhdl->data);
		if (ret)
			goto OUT;
	}

	if (mhdl->hdl_io_map_s1) {
		ret = arm_smmu_pgt_walk(cd->data, smmu_domain, mhdl);
		if (ret)
			goto OUT;
	}

OUT:
	if (mhdl->finish)
		mhdl->finish(mhdl->data, ret);

	mutex_unlock(&smmu->streams_mutex);
	return 0;
}

int smmu_master_walk_register(struct arm_smmu_master *master)
{
	struct smmu_master_info *info;

	if (!master)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->master = master;

	INIT_LIST_HEAD(&info->list);
	mutex_lock(&master_list_lock);
	list_add_tail(&info->list, &smmu_master_list);
	mutex_unlock(&master_list_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(smmu_master_walk_register);

void smmu_master_walk_unregister(struct arm_smmu_master *master)
{
	struct smmu_master_info *info, *next;

	if (!master)
		return;

	mutex_lock(&master_list_lock);
	list_for_each_entry_safe(info, next, &smmu_master_list, list)
		if (master == info->master)
			list_del(&info->list);
	mutex_unlock(&master_list_lock);

	return;
}
EXPORT_SYMBOL_GPL(smmu_master_walk_unregister);

int smmu_master_walk(struct smmu_master_handle *mhdl)
{
	struct smmu_master_info *info;

	mutex_lock(&master_list_lock);
	list_for_each_entry(info, &smmu_master_list, list)
		smmu_master_walk_one(info->master, mhdl);
	mutex_unlock(&master_list_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(smmu_master_walk);

static int __init smmu_info_init(void)
{
	mutex_init(&master_list_lock);
	return 0;
}
subsys_initcall(smmu_info_init)
