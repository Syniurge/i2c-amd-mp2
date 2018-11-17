// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * AMD MP2 PCIe communication driver
 *
 * Authors: Shyam Sundar S K <Shyam-sundar.S-k@amd.com>
 *          Elie Morisse <syniurge@gmail.com>
 */

#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "i2c-amd-mp2.h"

#define DRIVER_NAME	"i2c_amd_mp2"
#define DRIVER_DESC	"AMD(R) PCI-E MP2 I2C Controller Driver"
#define DRIVER_VER	"1.0"

#define write64 _write64
static inline void _write64(u64 val, void __iomem *mmio)
{
	writel(val, mmio);
	writel(val >> 32, mmio + sizeof(u32));
}

#define read64 _read64
static inline u64 _read64(void __iomem *mmio)
{
	u64 low, high;

	low = readl(mmio);
	high = readl(mmio + sizeof(u32));
	return low | (high << 32);
}

static void amd_mp2_c2p_mutex_lock(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;

	/* there is only one data mailbox for two i2c adapters */
	mutex_lock(&privdata->c2p_lock);
	privdata->c2p_lock_busid = i2c_common->bus_id;
}

static void amd_mp2_c2p_mutex_unlock(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;

	if (unlikely(privdata->c2p_lock_busid != i2c_common->bus_id)) {
		dev_warn(ndev_dev(privdata),
			 "bus %d attempting to unlock C2P locked by bus %d\n",
			 i2c_common->bus_id, privdata->c2p_lock_busid);
		return;
	}

	mutex_unlock(&privdata->c2p_lock);
}

static int amd_mp2_cmd(struct amd_mp2_dev *privdata,
		       union i2c_cmd_base i2c_cmd_base)
{
	void __iomem *reg;

	reg = privdata->mmio + ((i2c_cmd_base.s.bus_id == 1) ?
				AMD_C2P_MSG1 : AMD_C2P_MSG0);
	writel(i2c_cmd_base.ul, reg);

	return 0;
}

int amd_mp2_bus_xnable(struct amd_i2c_common *i2c_common, bool enable)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;
	union i2c_cmd_base i2c_cmd_base;

	dev_dbg(ndev_dev(privdata), "%s id: %d\n", __func__,
		i2c_common->bus_id);

	i2c_common->reqcmd = enable ? i2c_enable : i2c_disable;

	i2c_cmd_base.ul = 0;
	i2c_cmd_base.s.i2c_cmd = i2c_common->reqcmd;
	i2c_cmd_base.s.bus_id = i2c_common->bus_id;
	i2c_cmd_base.s.i2c_speed = i2c_common->i2c_speed;

	return amd_mp2_cmd(privdata, i2c_cmd_base);
}

static void amd_mp2_cmd_rw_fill(struct amd_i2c_common *i2c_common,
				union i2c_cmd_base *i2c_cmd_base,
				enum i2c_cmd reqcmd)
{
	i2c_common->reqcmd = reqcmd;

	i2c_cmd_base->s.i2c_cmd = reqcmd;
	i2c_cmd_base->s.bus_id = i2c_common->bus_id;
	i2c_cmd_base->s.i2c_speed = i2c_common->i2c_speed;
	i2c_cmd_base->s.slave_addr = i2c_common->msg->addr;
	i2c_cmd_base->s.length = i2c_common->msg->len;
}

#ifndef i2c_get_dma_safe_msg_buf
// DKMS only, for kernels older than 4.16, assume msg->buf to be DMA safe since
// I2C_M_DMA_SAFE wasn't introduced either
u8 *i2c_get_dma_safe_msg_buf(struct i2c_msg *msg, unsigned int threshold)
{
	/* ... */

	/* if (msg->flags & I2C_M_DMA_SAFE) */
		return msg->buf;

	/* ... */
}
#endif

static int amd_mp2_dma_map(struct amd_mp2_dev *privdata,
			   struct amd_i2c_common *i2c_common)
{
	enum dma_data_direction dma_direction =
			i2c_common->msg->flags & I2C_M_RD ?
			DMA_FROM_DEVICE : DMA_TO_DEVICE;

	i2c_common->dma_buf = i2c_get_dma_safe_msg_buf(i2c_common->msg, 0);
	i2c_common->dma_addr = dma_map_single(&privdata->pci_dev->dev,
					      i2c_common->dma_buf,
					      i2c_common->msg->len,
					      dma_direction);

	if (dma_mapping_error(&privdata->pci_dev->dev,
			      i2c_common->dma_addr)) {
		dev_err(ndev_dev(privdata),
			"Error while mapping dma buffer %p\n",
			i2c_common->dma_buf);
		return -EIO;
	}

	return 0;
}

#ifndef i2c_put_dma_safe_msg_buf
// DKMS only, for kernels older than 4.19
void i2c_put_dma_safe_msg_buf(u8 *buf, struct i2c_msg *msg, bool xferred)
{
	if (!buf || buf == msg->buf)
		return;

	if (xferred && msg->flags & I2C_M_RD)
		memcpy(msg->buf, buf, msg->len);

	kfree(buf);
}
#endif

static void amd_mp2_dma_unmap(struct amd_mp2_dev *privdata,
			      struct amd_i2c_common *i2c_common)
{
	enum dma_data_direction dma_direction =
			i2c_common->msg->flags & I2C_M_RD ?
			DMA_FROM_DEVICE : DMA_TO_DEVICE;

	dma_unmap_single(&privdata->pci_dev->dev,
			 i2c_common->dma_addr,
			 i2c_common->msg->len,
			 dma_direction);

	i2c_put_dma_safe_msg_buf(i2c_common->dma_buf, i2c_common->msg, true);
}

int amd_mp2_read(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;
	union i2c_cmd_base i2c_cmd_base;

	dev_dbg(ndev_dev(privdata), "%s addr: %x id: %d\n", __func__,
		i2c_common->msg->addr, i2c_common->bus_id);

	amd_mp2_cmd_rw_fill(i2c_common, &i2c_cmd_base, i2c_read);
	amd_mp2_c2p_mutex_lock(i2c_common);

	if (i2c_common->msg->len <= 32) {
		i2c_cmd_base.s.mem_type = use_c2pmsg;
	} else {
		i2c_cmd_base.s.mem_type = use_dram;
		if (amd_mp2_dma_map(privdata, i2c_common))
			return -EIO;
		write64((u64)i2c_common->dma_addr,
			privdata->mmio + AMD_C2P_MSG2);
	}

	return amd_mp2_cmd(privdata, i2c_cmd_base);
}

int amd_mp2_write(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;
	union i2c_cmd_base i2c_cmd_base;

	dev_dbg(ndev_dev(privdata), "%s addr: %x id: %d\n", __func__,
		i2c_common->msg->addr, i2c_common->bus_id);

	amd_mp2_cmd_rw_fill(i2c_common, &i2c_cmd_base, i2c_write);
	amd_mp2_c2p_mutex_lock(i2c_common);

	if (i2c_common->msg->len <= 32) {
		i2c_cmd_base.s.mem_type = use_c2pmsg;
		memcpy_toio(privdata->mmio + AMD_C2P_MSG2,
			    i2c_common->msg->buf,
			    i2c_common->msg->len);
	} else {
		i2c_cmd_base.s.mem_type = use_dram;
		if (amd_mp2_dma_map(privdata, i2c_common))
			return -EIO;
		write64((u64)i2c_common->dma_addr,
			privdata->mmio + AMD_C2P_MSG2);
	}

	return amd_mp2_cmd(privdata, i2c_cmd_base);
}

static void amd_mp2_pci_check_rw_event(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;
	int len = i2c_common->eventval.r.length;
	u32 slave_addr = i2c_common->eventval.r.slave_addr;
	bool err = false;

	if (len != i2c_common->msg->len) {
		dev_err(ndev_dev(privdata),
			"length %d in event doesn't match buffer length %d!\n",
			len, i2c_common->msg->len);
		err = true;
	}

	if (slave_addr != i2c_common->msg->addr) {
		dev_err(ndev_dev(privdata),
			"unexpected slave address %x (expected: %x)!\n",
			slave_addr, i2c_common->msg->addr);
		err = true;
	}

	if (!err)
		i2c_common->cmd_success = true;
}

static void amd_mp2_pci_do_work(struct work_struct *work)
{
	struct amd_i2c_common *i2c_common = work_amd_i2c_common(work);
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;
	int sts = i2c_common->eventval.r.status;
	int res = i2c_common->eventval.r.response;
	int len = i2c_common->eventval.r.length;

	if ((i2c_common->reqcmd == i2c_read ||
	     i2c_common->reqcmd == i2c_write) &&
	    i2c_common->msg->len > 32)
		amd_mp2_dma_unmap(privdata, i2c_common);

	if (res != command_success) {
		if (res == command_failed)
			dev_err(ndev_dev(privdata), "i2c command failed!\n");
		else
			dev_err(ndev_dev(privdata), "invalid response to i2c command!\n");
		return;
	}

	switch (i2c_common->reqcmd) {
	case i2c_read:
		if (sts == i2c_readcomplete_event) {
			amd_mp2_pci_check_rw_event(i2c_common);
			if (len <= 32)
				memcpy_fromio(i2c_common->msg->buf,
					      privdata->mmio + AMD_C2P_MSG2,
					      i2c_common->msg->len);
		} else if (sts == i2c_readfail_event) {
			dev_err(ndev_dev(privdata), "i2c read failed!\n");
		} else {
			dev_err(ndev_dev(privdata),
				"invalid i2c status after read (%d)!\n", sts);
		}
		break;
	case i2c_write:
		if (sts == i2c_writecomplete_event)
			amd_mp2_pci_check_rw_event(i2c_common);
		else if (sts == i2c_writefail_event)
			dev_err(ndev_dev(privdata), "i2c write failed!\n");
		else
			dev_err(ndev_dev(privdata),
				"invalid i2c status after write (%d)!\n", sts);
		break;
	case i2c_enable:
		if (sts == i2c_busenable_failed)
			dev_err(ndev_dev(privdata), "i2c bus enable failed!\n");
		else if (sts != i2c_busenable_complete)
			dev_err(ndev_dev(privdata),
				"invalid i2c status after bus enable (%d)!\n",
				sts);
		else
			i2c_common->cmd_success = true;
		break;
	case i2c_disable:
		if (sts == i2c_busdisable_failed)
			dev_err(ndev_dev(privdata), "i2c bus disable failed!\n");
		else if (sts != i2c_busdisable_complete)
			dev_err(ndev_dev(privdata),
				"invalid i2c status after bus disable (%d)!\n",
				sts);
		else
			i2c_common->cmd_success = true;
		break;
	default:
		break;
	}
}

static void amd_mp2_pci_work(struct work_struct *work)
{
	struct amd_i2c_common *i2c_common = work_amd_i2c_common(work);
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;
	enum i2c_cmd cmd = i2c_common->reqcmd;

	amd_mp2_pci_do_work(work);

	i2c_common->reqcmd = i2c_none;
	i2c_amd_msg_completion(i2c_common);

	synchronize_irq(privdata->pci_dev->irq);
	writel(0, privdata->mmio + AMD_P2C_MSG_INTEN);

	if (cmd == i2c_read || cmd == i2c_write)
		mutex_unlock(&privdata->c2p_lock);
}

static irqreturn_t amd_mp2_irq_isr(int irq, void *dev)
{
	struct amd_mp2_dev *privdata = dev;
	struct amd_i2c_common *i2c_common;
	u32 val;
	unsigned int bus_id;
	void __iomem *reg;
	enum irqreturn ret = IRQ_NONE;

	for (bus_id = 0; bus_id < 2; bus_id++) {
		i2c_common = privdata->busses[bus_id];
		if (!i2c_common || i2c_common->reqcmd == i2c_none)
			continue;

		reg = privdata->mmio + ((bus_id == 0) ?
					AMD_P2C_MSG1 : AMD_P2C_MSG2);
		val = readl(reg);
		if (val != 0) {
			writel(0, reg);
			i2c_common->eventval.ul = val;
			schedule_delayed_work(&i2c_common->work, 0);
			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

void amd_mp2_rw_timeout(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;

	i2c_common->reqcmd = i2c_none;
	writel(0, privdata->mmio + AMD_P2C_MSG_INTEN);
	amd_mp2_c2p_mutex_unlock(i2c_common);
}

int amd_mp2_register_cb(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;

	if (i2c_common->bus_id > 1)
		return -EINVAL;

	if (privdata->busses[i2c_common->bus_id]) {
		dev_err(ndev_dev(privdata),
			"Bus %d already taken!\n", i2c_common->bus_id);
		return -EINVAL;
	}

	privdata->busses[i2c_common->bus_id] = i2c_common;
	i2c_common->reqcmd = i2c_none;
	INIT_DELAYED_WORK(&i2c_common->work, amd_mp2_pci_work);

	return 0;
}

int amd_mp2_unregister_cb(struct amd_i2c_common *i2c_common)
{
	struct amd_mp2_dev *privdata = i2c_common->mp2_dev;

	cancel_delayed_work_sync(&i2c_common->work);
	privdata->busses[i2c_common->bus_id] = NULL;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static const struct file_operations amd_mp2_debugfs_info;
static struct dentry *debugfs_root_dir;

static ssize_t amd_mp2_debugfs_read(struct file *filp, char __user *ubuf,
				    size_t count, loff_t *offp)
{
	struct amd_mp2_dev *privdata;
	void __iomem *mmio;
	u8 *buf;
	size_t buf_size;
	ssize_t ret, off = 0;
	u32 v32;
	int i;

	privdata = filp->private_data;
	mmio = privdata->mmio;
	buf_size = min_t(size_t, count, 0x800);
	buf = kmalloc(buf_size, GFP_KERNEL);

	if (!buf)
		return -ENOMEM;

	off += scnprintf(buf + off, buf_size - off,
			 "MP2 Device Information:\n");

	off += scnprintf(buf + off, buf_size - off,
			 "========================\n");
	off += scnprintf(buf + off, buf_size - off,
			 "\tMP2 C2P Message Register Dump:\n\n");

	for (i = 0; i < 10; i++) {
		v32 = readl(privdata->mmio + AMD_C2P_MSG0 + i * 4);
		off += scnprintf(buf + off, buf_size - off,
				 "AMD_C2P_MSG%d -\t\t\t%#06x\n", i, v32);
	}

	off += scnprintf(buf + off, buf_size - off,
			"\n\tMP2 P2C Message Register Dump:\n\n");

	for (i = 0; i < 3; i++) {
		v32 = readl(privdata->mmio + AMD_P2C_MSG1 + i * 4);
		off += scnprintf(buf + off, buf_size - off,
				 "AMD_P2C_MSG%d -\t\t\t%#06x\n", i + 1, v32);
	}

	v32 = readl(privdata->mmio + AMD_P2C_MSG_INTEN);
	off += scnprintf(buf + off, buf_size - off,
			"AMD_P2C_MSG_INTEN -\t\t%#06x\n", v32);

	v32 = readl(privdata->mmio + AMD_P2C_MSG_INTSTS);
	off += scnprintf(buf + off, buf_size - off,
			"AMD_P2C_MSG_INTSTS -\t\t%#06x\n", v32);

	ret = simple_read_from_buffer(ubuf, count, offp, buf, off);
	kfree(buf);
	return ret;
}

static const struct file_operations amd_mp2_debugfs_info = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = amd_mp2_debugfs_read,
};

static void amd_mp2_init_debugfs(struct amd_mp2_dev *privdata)
{
	if (!debugfs_root_dir)
		return;

	privdata->debugfs_dir = debugfs_create_dir(ndev_name(privdata),
						   debugfs_root_dir);
	if (!privdata->debugfs_dir) {
		privdata->debugfs_info = NULL;
	} else {
		privdata->debugfs_info = debugfs_create_file
			("info", 0400, privdata->debugfs_dir,
			 privdata, &amd_mp2_debugfs_info);
	}
}
#endif /* CONFIG_DEBUG_FS */

static void amd_mp2_clear_reg(struct amd_mp2_dev *privdata)
{
	int reg;

	for (reg = AMD_C2P_MSG0; reg <= AMD_C2P_MSG9; reg += 4)
		writel(0, privdata->mmio + reg);

	for (reg = AMD_P2C_MSG1; reg <= AMD_P2C_MSG2; reg += 4)
		writel(0, privdata->mmio + reg);
}

static int amd_mp2_pci_init(struct amd_mp2_dev *privdata,
			    struct pci_dev *pci_dev)
{
	int rc;

	pci_set_drvdata(pci_dev, privdata);

	rc = pcim_enable_device(pci_dev);
	if (rc) {
		dev_err(ndev_dev(privdata), "Failed to enable MP2 PCI device\n");
		goto err_pci_enable;
	}

	rc = pcim_iomap_regions(pci_dev, 1 << 2, pci_name(pci_dev));
	if (rc) {
		dev_err(ndev_dev(privdata), "I/O memory remapping failed\n");
		goto err_pci_enable;
	}
	privdata->mmio = pcim_iomap_table(pci_dev)[2];

	pci_set_master(pci_dev);

	rc = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64));
	if (rc) {
		rc = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));
		if (rc)
			goto err_dma_mask;
	}

	/* Set up intx irq */
	writel(0, privdata->mmio + AMD_P2C_MSG_INTEN);
	raw_spin_lock_init(&privdata->lock);
	pci_intx(pci_dev, 1);
	rc = devm_request_irq(&pci_dev->dev, pci_dev->irq, amd_mp2_irq_isr,
			      IRQF_SHARED, dev_name(&pci_dev->dev), privdata);
	if (rc)
		dev_err(&pci_dev->dev, "Failure requesting irq %i: %d\n",
			pci_dev->irq, rc);

	return rc;

err_dma_mask:
	pci_clear_master(pci_dev);
err_pci_enable:
	pci_set_drvdata(pci_dev, NULL);
	return rc;
}

static int amd_mp2_pci_probe(struct pci_dev *pci_dev,
			     const struct pci_device_id *id)
{
	struct amd_mp2_dev *privdata;
	int rc;
	static bool first_probe = true;

	if (first_probe) {
		pr_info("%s: %s Version: %s\n", DRIVER_NAME,
			DRIVER_DESC, DRIVER_VER);
		first_probe = false;
	}

	dev_info(&pci_dev->dev, "MP2 device found [%04x:%04x] (rev %x)\n",
		 pci_dev->vendor, pci_dev->device, pci_dev->revision);

	privdata = devm_kzalloc(&pci_dev->dev, sizeof(*privdata), GFP_KERNEL);
	if (!privdata)
		return -ENOMEM;

	rc = amd_mp2_pci_init(privdata, pci_dev);
	if (rc)
		return rc;

	mutex_init(&privdata->c2p_lock);
	privdata->pci_dev = pci_dev;

	amd_mp2_init_debugfs(privdata);
	dev_info(&pci_dev->dev, "MP2 device registered.\n");
	return 0;
}

static bool amd_mp2_pci_is_probed(struct pci_dev *pci_dev)
{
	struct amd_mp2_dev *privdata = pci_get_drvdata(pci_dev);

	if (!privdata)
		return false;
	return privdata->pci_dev != NULL;
}

static void amd_mp2_pci_remove(struct pci_dev *pci_dev)
{
	struct amd_mp2_dev *privdata = pci_get_drvdata(pci_dev);
	unsigned int bus_id;

	for (bus_id = 0; bus_id < 2; bus_id++)
		if (privdata->busses[bus_id])
			i2c_amd_delete_adapter(privdata->busses[bus_id]);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(privdata->debugfs_dir);
#endif /* CONFIG_DEBUG_FS */

	pci_intx(pci_dev, 0);
	pci_clear_master(pci_dev);

	amd_mp2_clear_reg(privdata);
}

static const struct pci_device_id amd_mp2_pci_tbl[] = {
	{PCI_VDEVICE(AMD, PCI_DEVICE_ID_AMD_MP2)},
	{0}
};
MODULE_DEVICE_TABLE(pci, amd_mp2_pci_tbl);

// #ifdef CONFIG_PM_SLEEP
// static int amd_mp2_pci_device_suspend(struct pci_dev *pci_dev,
// 				      pm_message_t mesg)
// {
// 	int ret;
// 	struct amd_mp2_dev *privdata = pci_get_drvdata(pci_dev);
//
// 	if (!privdata)
// 		return -EINVAL;
//
// 	ret = pci_save_state(pci_dev);
// 	if (ret) {
// 		dev_err(ndev_dev(privdata),
// 			"pci_save_state failed = %d\n", ret);
// 		return ret;
// 	}
//
// 	pci_enable_wake(pci_dev, PCI_D3hot, 0);
// 	pci_disable_device(pci_dev);
// 	pci_set_power_state(pci_dev, pci_choose_state(pci_dev, mesg));
//
// 	return 0;
// }
//
// static int amd_mp2_pci_device_resume(struct pci_dev *pci_dev)
// {
// 	struct amd_mp2_dev *privdata = pci_get_drvdata(pci_dev);
//
// 	if (!privdata)
// 		return -EINVAL;
//
// 	pci_set_power_state(pci_dev, PCI_D0);
// 	pci_restore_state(pci_dev);
//
// 	if (pci_enable_device(pci_dev) < 0) {
// 		dev_err(ndev_dev(privdata), "pci_enable_device failed\n");
// 		return -EIO;
// 	}
//
// 	pci_enable_wake(pci_dev, PCI_D3hot, 0);
//
// 	return 0;
// }
// #endif

static struct pci_driver amd_mp2_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= amd_mp2_pci_tbl,
	.probe		= amd_mp2_pci_probe,
	.remove		= amd_mp2_pci_remove,
// #ifdef CONFIG_PM_SLEEP
// 	.suspend	= amd_mp2_pci_device_suspend,
// 	.resume		= amd_mp2_pci_device_resume,
// #endif
};

static int amd_mp2_device_match(struct device *dev, void *data)
{
	struct pci_dev *candidate = data;

	if (!candidate)
		return 1;
	return (to_pci_dev(dev) == candidate) ? 1 : 0;
}

struct amd_mp2_dev *amd_mp2_find_device(struct pci_dev *candidate)
{
	struct device *dev;
	struct pci_dev *pci_dev;

	dev = driver_find_device(&amd_mp2_pci_driver.driver, NULL, candidate,
				 amd_mp2_device_match);
	if (!dev)
		return NULL;

	pci_dev = to_pci_dev(dev);
	if (!amd_mp2_pci_is_probed(pci_dev))
		return NULL;
	return (struct amd_mp2_dev *)pci_get_drvdata(pci_dev);
}

static int __init amd_mp2_drv_init(void)
{
	int rc;

#ifdef CONFIG_DEBUG_FS
	debugfs_root_dir = debugfs_create_dir(KBUILD_MODNAME, NULL);
#endif /* CONFIG_DEBUG_FS */

	rc = pci_register_driver(&amd_mp2_pci_driver);
	if (rc)
		return rc;
	return i2c_amd_register_driver();
}
module_init(amd_mp2_drv_init);

static void __exit amd_mp2_drv_exit(void)
{
	i2c_amd_unregister_driver();
	pci_unregister_driver(&amd_mp2_pci_driver);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(debugfs_root_dir);
#endif /* CONFIG_DEBUG_FS */
}
module_exit(amd_mp2_drv_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VER);
MODULE_AUTHOR("Shyam Sundar S K <Shyam-sundar.S-k@amd.com>");
MODULE_AUTHOR("Nehal Shah <nehal-bakulchandra.shah@amd.com>");
MODULE_AUTHOR("Elie Morisse <syniurge@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
