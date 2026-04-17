// SPDX-License-Identifier: GPL-2.0
/*
 * For transport using shared mem structure.
 *
 * Copyright (C) 2019-2024 ARM Ltd.
 */

#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/property.h>
#include <linux/types.h>

#include <linux/bug.h>

#include "common.h"

#define SCMI_SHMEM_LAYOUT_OVERHEAD	24

/*
 * SCMI specification requires all parameters, message headers, return
 * arguments or any protocol data to be expressed in little endian
 * format only.
 */
#ifndef CONFIG_PM_EXCEPTION_PROTOCOL
struct scmi_shared_mem {
	__le32 reserved;
	__le32 channel_status;
#define SCMI_SHMEM_CHAN_STAT_CHANNEL_ERROR	BIT(1)
#define SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE	BIT(0)
	__le32 reserved1[2];
	__le32 flags;
#define SCMI_SHMEM_FLAG_INTR_ENABLED	BIT(0)
	__le32 length;
	__le32 msg_header;
	u8 msg_payload[];
};
#else

#define SCP_PM_MSG_LEN (0x7f)
#define PM_MSG_MAX_LEN_TX (12 * 4)
#define PM_MSG_MAX_LEN_RX (13 * 4)

struct scmi_shared_mem {
	union {
		__le32 reserved;
		struct {
			__le32 buf_len	: 7;
			__le32 rsvd0	: 1;
			__le32 msg_route: 8;
			__le32 rsvd1	: 16;
		} msg_info;
	};
	__le32 channel_status;
#define SCMI_SHMEM_CHAN_STAT_CHANNEL_ERROR	BIT(1)
#define SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE	BIT(0)
	union {
		__le32 reserved1[2];
		struct {
			__le32 statusCode;
			__le32 traceCode;
		} stCode;
	};
	__le32 flags;
#define SCMI_SHMEM_FLAG_INTR_ENABLED	BIT(0)
	__le32 length;
	union {
		__le32 msg_header;
		struct {
			__le32 msgID	: 16;
			__le32 rsvd0	: 2;
			__le32 token	: 10;
			__le32 rsvd1	: 4;
		} pm_header;
	};
	u8 msg_payload[];
};
#endif

static inline void shmem_memcpy_fromio32(void *to,
					 const void __iomem *from,
					 size_t count)
{
	WARN_ON(!IS_ALIGNED((unsigned long)from, 4) ||
		!IS_ALIGNED((unsigned long)to, 4) ||
		count % 4);

	__ioread32_copy(to, from, count / 4);
}

static inline void shmem_memcpy_toio32(void __iomem *to,
				       const void *from,
				       size_t count)
{
	WARN_ON(!IS_ALIGNED((unsigned long)to, 4) ||
		!IS_ALIGNED((unsigned long)from, 4) ||
		count % 4);

	__iowrite32_copy(to, from, count / 4);
}

static struct scmi_shmem_io_ops shmem_io_ops32 = {
	.fromio	= shmem_memcpy_fromio32,
	.toio	= shmem_memcpy_toio32,
};

/* Wrappers are needed for proper memcpy_{from,to}_io expansion by the
 * pre-processor.
 */
static inline void shmem_memcpy_fromio(void *to,
				       const void __iomem *from,
				       size_t count)
{
	memcpy_fromio(to, from, count);
}

static inline void shmem_memcpy_toio(void __iomem *to,
				     const void *from,
				     size_t count)
{
	memcpy_toio(to, from, count);
}

static struct scmi_shmem_io_ops shmem_io_ops_default = {
	.fromio = shmem_memcpy_fromio,
	.toio	= shmem_memcpy_toio,
};

static void shmem_tx_prepare(struct scmi_shared_mem __iomem *shmem,
			     struct scmi_xfer *xfer,
			     struct scmi_chan_info *cinfo,
			     shmem_copy_toio_t copy_toio)
{
	ktime_t stop;
#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
	bool isCustomized;

	if (xfer->hdr.protocol_id == 0x81)
		isCustomized = true;
	else
		isCustomized = false;
#endif

	/*
	 * Ideally channel must be free by now unless OS timeout last
	 * request and platform continued to process the same, wait
	 * until it releases the shared memory, otherwise we may endup
	 * overwriting its response with new message payload or vice-versa.
	 * Giving up anyway after twice the expected channel timeout so as
	 * not to bail-out on intermittent issues where the platform is
	 * occasionally a bit slower to answer.
	 *
	 * Note that after a timeout is detected we bail-out and carry on but
	 * the transport functionality is probably permanently compromised:
	 * this is just to ease debugging and avoid complete hangs on boot
	 * due to a misbehaving SCMI firmware.
	 */
	stop = ktime_add_ms(ktime_get(), 2 * cinfo->rx_timeout_ms);
	spin_until_cond((ioread32(&shmem->channel_status) &
			 SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE) ||
			 ktime_after(ktime_get(), stop));
	if (!(ioread32(&shmem->channel_status) &
	      SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE)) {
		dev_warn(cinfo->dev,
			"Timeout waiting for a free TX channel, forcing clear\n");
		/* Force-clear the stuck channel so SCMI can recover.
		 * The previous message response will be lost, but this
		 * prevents permanent SCMI communication failure.
		 */
		iowrite32(SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE,
			  &shmem->channel_status);
	}

	/* Mark channel busy + clear error */
	iowrite32(0x0, &shmem->channel_status);
	/* Ensure status write is visible before payload writes */
	wmb();
	iowrite32(xfer->hdr.poll_completion ? 0 : SCMI_SHMEM_FLAG_INTR_ENABLED,
		  &shmem->flags);
	iowrite32(sizeof(shmem->msg_header) + xfer->tx.len, &shmem->length);
	iowrite32(pack_scmi_header(&xfer->hdr), &shmem->msg_header);
#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
	if (isCustomized)
		iowrite32(0xc7f, &shmem->msg_info);
	else
		iowrite32(0x0, &shmem->reserved);
#endif
	if (xfer->tx.buf) {
#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
		if (isCustomized)
			if (xfer->tx.len > PM_MSG_MAX_LEN_TX)
				pr_err("TX Length is OVERSIZE!\n");
#endif
#ifdef CONFIG_ARCH_CIX
		/*
		 * CIX Sky1 SCP firmware requires 32-bit aligned MMIO writes
		 * to shared memory — memcpy_toio may use byte-width stores
		 * on ARM64, which the SCP silently ignores.
		 */
		{
			int i;

			for (i = 0; i < DIV_ROUND_UP(xfer->tx.len, 4); i++)
				__raw_writel(((u32 *)xfer->tx.buf)[i],
					     shmem->msg_payload + 4 * i);
		}
#else
		copy_toio(shmem->msg_payload, xfer->tx.buf, xfer->tx.len);
#endif
	}
	/* Ensure all payload writes complete before signaling firmware */
	wmb();
}

static u32 shmem_read_header(struct scmi_shared_mem __iomem *shmem)
{
	return ioread32(&shmem->msg_header);
}

static void shmem_fetch_response(struct scmi_shared_mem __iomem *shmem,
				 struct scmi_xfer *xfer,
				 shmem_copy_fromio_t copy_fromio)
{
	size_t len = ioread32(&shmem->length);

#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
	bool isCustomized;

	if (xfer->hdr.protocol_id == 0x81)
		isCustomized = true;
	else
		isCustomized = false;

	if (isCustomized)
		xfer->hdr.status = ioread32(&shmem->stCode.statusCode);
	else
		xfer->hdr.status = ioread32(shmem->msg_payload);
#else
	xfer->hdr.status = ioread32(shmem->msg_payload);
#endif
#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
	if (isCustomized) {
		xfer->rx.len = min_t(size_t, xfer->rx.len, len > 4 ? len - 4 : 0);
		if (xfer->rx.len > PM_MSG_MAX_LEN_RX)
			pr_err("RX Length is OVERSIZE!\n");
	} else {
		/* Skip the length of header and status in shmem area i.e 8 bytes */
		xfer->rx.len = min_t(size_t, xfer->rx.len, len > 8 ? len - 8 : 0);
	}
#else
	/* Skip the length of header and status in shmem area i.e 8 bytes */
	xfer->rx.len = min_t(size_t, xfer->rx.len, len > 8 ? len - 8 : 0);
#endif

	/* Take a copy to the rx buffer.. */
#ifdef CONFIG_ARCH_CIX
	{
		int i;
#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
		int off = isCustomized ? 12 : 1;
#else
		int off = 1;
#endif
		for (i = 0; i < DIV_ROUND_UP(xfer->rx.len, 4); i++)
			((u32 *)xfer->rx.buf)[i] =
				ioread32(shmem->msg_payload + 4 * (i + off));
	}
#else
#ifdef CONFIG_PM_EXCEPTION_PROTOCOL
	if (isCustomized)
		copy_fromio(xfer->rx.buf, shmem->msg_payload + 4 * 12, xfer->rx.len);
	else
		copy_fromio(xfer->rx.buf, shmem->msg_payload + 4, xfer->rx.len);
#else
	copy_fromio(xfer->rx.buf, shmem->msg_payload + 4, xfer->rx.len);
#endif
#endif
}

static void shmem_fetch_notification(struct scmi_shared_mem __iomem *shmem,
				     size_t max_len, struct scmi_xfer *xfer,
				     shmem_copy_fromio_t copy_fromio)
{
	size_t len = ioread32(&shmem->length);

	/* Skip only the length of header in shmem area i.e 4 bytes */
	xfer->rx.len = min_t(size_t, max_len, len > 4 ? len - 4 : 0);

	/* Take a copy to the rx buffer.. */
#ifdef CONFIG_ARCH_CIX
	{
		int i;

		for (i = 0; i < DIV_ROUND_UP(xfer->rx.len, 4); i++)
			((u32 *)xfer->rx.buf)[i] =
				ioread32(shmem->msg_payload + 4 * i);
	}
#else
	copy_fromio(xfer->rx.buf, shmem->msg_payload, xfer->rx.len);
#endif
}

static void shmem_clear_channel(struct scmi_shared_mem __iomem *shmem)
{
	iowrite32(SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE, &shmem->channel_status);
}

static bool shmem_poll_done(struct scmi_shared_mem __iomem *shmem,
			    struct scmi_xfer *xfer)
{
	u16 xfer_id;

	xfer_id = MSG_XTRACT_TOKEN(ioread32(&shmem->msg_header));

	if (xfer->hdr.seq != xfer_id)
		return false;

	return ioread32(&shmem->channel_status) &
		(SCMI_SHMEM_CHAN_STAT_CHANNEL_ERROR |
		 SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE);
}

static bool shmem_channel_free(struct scmi_shared_mem __iomem *shmem)
{
	return (ioread32(&shmem->channel_status) &
			SCMI_SHMEM_CHAN_STAT_CHANNEL_FREE);
}

static bool shmem_channel_intr_enabled(struct scmi_shared_mem __iomem *shmem)
{
	return ioread32(&shmem->flags) & SCMI_SHMEM_FLAG_INTR_ENABLED;
}

static void __iomem *shmem_setup_iomap(struct scmi_chan_info *cinfo,
				       struct device *dev, bool tx,
				       struct resource *res,
				       struct scmi_shmem_io_ops **ops)
{
	const char *desc = tx ? "Tx" : "Rx";
	int ret, idx = tx ? 0 : 1;
	struct device *cdev = cinfo->dev;
	struct resource lres = {};
	resource_size_t size;
	void __iomem *addr;
	u32 reg_io_width = 0;

	/* Use a local on-stack as a working area when not provided */
	if (!res)
		res = &lres;

	if (cdev->of_node) {
		struct device_node *shmem __free(device_node) =
			of_parse_phandle(cdev->of_node, "shmem", idx);

		if (!shmem)
			return IOMEM_ERR_PTR(-ENODEV);

		if (!of_device_is_compatible(shmem, "arm,scmi-shmem"))
			return IOMEM_ERR_PTR(-ENXIO);

		ret = of_address_to_resource(shmem, 0, res);
		if (ret) {
			dev_err(cdev, "failed to get SCMI %s shared memory\n",
				desc);
			return IOMEM_ERR_PTR(ret);
		}

		of_property_read_u32(shmem, "reg-io-width", &reg_io_width);
	} else {
		struct fwnode_handle *shmem_fwnode;
		struct device *rdev;
		struct platform_device *pdev;
		const char *compat;

		shmem_fwnode = fwnode_find_reference(cdev->fwnode, "shmem", idx);
		if (IS_ERR_OR_NULL(shmem_fwnode))
			return IOMEM_ERR_PTR(-ENODEV);

		if (fwnode_property_present(shmem_fwnode, "compatible")) {
			ret = fwnode_property_read_string(shmem_fwnode,
				"compatible", &compat);
			if (ret || strcmp(compat, "arm,scmi-shmem")) {
				fwnode_handle_put(shmem_fwnode);
				return IOMEM_ERR_PTR(-ENXIO);
			}
		}

		rdev = bus_find_device_by_fwnode(&platform_bus_type,
						 shmem_fwnode);
		fwnode_handle_put(shmem_fwnode);
		pdev = rdev ? to_platform_device(rdev) : NULL;
		if (!pdev) {
			dev_err(cdev, "failed to find SCMI %s shmem device\n",
				desc);
			if (rdev)
				put_device(rdev);
			return IOMEM_ERR_PTR(-ENODEV);
		}

		ret = platform_get_resource(pdev, IORESOURCE_MEM, 0) ?
			0 : -EINVAL;
		if (!ret) {
			struct resource *pres;

			pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
			*res = *pres;
		}
		put_device(rdev);
		if (ret) {
			dev_err(cdev, "failed to get SCMI %s shared memory\n",
				desc);
			return IOMEM_ERR_PTR(ret);
		}
	}

	size = resource_size(res);
	if (cinfo->max_msg_size + SCMI_SHMEM_LAYOUT_OVERHEAD > size) {
		dev_err(dev, "misconfigured SCMI shared memory\n");
		return IOMEM_ERR_PTR(-ENOSPC);
	}

	addr = devm_ioremap(dev, res->start, size);
	if (!addr) {
		dev_err(dev, "failed to ioremap SCMI %s shared memory\n", desc);
		return IOMEM_ERR_PTR(-EADDRNOTAVAIL);
	}

	switch (reg_io_width) {
	case 4:
		*ops = &shmem_io_ops32;
		break;
	default:
		*ops = &shmem_io_ops_default;
		break;
	}

	return addr;
}

static const struct scmi_shared_mem_operations scmi_shmem_ops = {
	.tx_prepare = shmem_tx_prepare,
	.read_header = shmem_read_header,
	.fetch_response = shmem_fetch_response,
	.fetch_notification = shmem_fetch_notification,
	.clear_channel = shmem_clear_channel,
	.poll_done = shmem_poll_done,
	.channel_free = shmem_channel_free,
	.channel_intr_enabled = shmem_channel_intr_enabled,
	.setup_iomap = shmem_setup_iomap,
};

const struct scmi_shared_mem_operations *scmi_shared_mem_operations_get(void)
{
	return &scmi_shmem_ops;
}
