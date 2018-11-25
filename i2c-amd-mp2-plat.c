// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * AMD MP2 platform driver
 *
 * Setup the I2C adapters enumerated in the ACPI namespace.
 * MP2 controllers have 2 separate buses, i.e up to 2 I2C adapters.
 *
 * Authors: Nehal Bakulchandra Shah <Nehal-bakulchandra.shah@amd.com>
 *          Elie Morisse <syniurge@gmail.com>
 */

#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "i2c-amd-mp2.h"

#define AMD_MP2_I2C_MAX_RW_LENGTH ((1 << 12) - 1)
#define AMD_I2C_TIMEOUT (msecs_to_jiffies(250))

/**
 * struct amd_i2c_dev - MP2 bus/i2c adapter context
 * @i2c_common: shared context with the MP2 PCI driver
 * @pdev: platform driver node
 * @adapter: i2c adapter
 * @completion: xfer completion object
 */
struct amd_i2c_dev {
	struct amd_i2c_common i2c_common;
	struct platform_device *pdev;
	struct i2c_adapter adapter;
	struct completion msg_complete;
};

#define amd_i2c_dev_common(__common) \
	container_of(__common, struct amd_i2c_dev, i2c_common)

void i2c_amd_msg_completion(struct amd_i2c_common *i2c_common)
{
	struct amd_i2c_dev *i2c_dev = amd_i2c_dev_common(i2c_common);
	union i2c_event *event = &i2c_common->eventval;

	if (event->r.status == i2c_readcomplete_event)
		dev_dbg(&i2c_dev->pdev->dev, "%s readdata:%*ph\n",
			__func__, event->r.length,
			i2c_common->msg->buf);

	complete(&i2c_dev->msg_complete);
}

static const char *i2c_amd_cmd_name(enum i2c_cmd cmd)
{
	switch (cmd) {
	case i2c_read:
		return "i2c read";
	case i2c_write:
		return "i2c write";
	case i2c_enable:
		return "bus enable";
	case i2c_disable:
		return "bus disable";
	case number_of_sensor_discovered:
		return "'number of sensors discovered' cmd";
	case is_mp2_active:
		return "'is mp2 active' cmd";
	default:
		return "invalid cmd";
	}
}

static int i2c_amd_check_msg_completion(struct amd_i2c_dev *i2c_dev)
{
	struct amd_i2c_common *i2c_common = &i2c_dev->i2c_common;
	unsigned long timeout;

	timeout = wait_for_completion_timeout(&i2c_dev->msg_complete,
					      AMD_I2C_TIMEOUT);
	if (timeout == 0) {
		dev_err(&i2c_dev->pdev->dev, "%s timed out\n",
			i2c_amd_cmd_name(i2c_common->reqcmd));
		amd_mp2_rw_timeout(i2c_common);
		return -ETIMEDOUT;
	}

	if (!i2c_common->cmd_success)
		return -EINVAL;

	return 0;
}

static int i2c_amd_xnable(struct amd_i2c_dev *i2c_dev, bool enable)
{
	struct amd_i2c_common *i2c_common = &i2c_dev->i2c_common;

	reinit_completion(&i2c_dev->msg_complete);
	amd_mp2_bus_xnable(i2c_common, enable);

	return i2c_amd_check_msg_completion(i2c_dev);
}

static int i2c_amd_xfer_msg(struct amd_i2c_dev *i2c_dev, struct i2c_msg *pmsg)
{
	struct amd_i2c_common *i2c_common = &i2c_dev->i2c_common;

	reinit_completion(&i2c_dev->msg_complete);

	i2c_common->msg = pmsg;
	i2c_common->cmd_success = false;

	if (pmsg->flags & I2C_M_RD)
		amd_mp2_read(i2c_common);
	else
		amd_mp2_write(i2c_common);

	return i2c_amd_check_msg_completion(i2c_dev);
}

static int i2c_amd_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct amd_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int i;
	struct i2c_msg *pmsg;
	int err;

	amd_mp2_pm_runtime_get(i2c_dev->i2c_common.mp2_dev);

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		err = i2c_amd_xfer_msg(i2c_dev, pmsg);
		if (err)
			break;
	}

	amd_mp2_pm_runtime_put(i2c_dev->i2c_common.mp2_dev);
	return err ? err : num;
}

static u32 i2c_amd_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm i2c_amd_algorithm = {
	.master_xfer = i2c_amd_xfer,
	.functionality = i2c_amd_func,
};

#ifdef CONFIG_PM
int i2c_amd_suspend(struct amd_i2c_common *i2c_common)
{
	struct amd_i2c_dev *i2c_dev = amd_i2c_dev_common(i2c_common);

	i2c_amd_xnable(i2c_dev, false);
	return 0;
}

int i2c_amd_resume(struct amd_i2c_common *i2c_common)
{
	struct amd_i2c_dev *i2c_dev = amd_i2c_dev_common(i2c_common);

	return i2c_amd_xnable(i2c_dev, true);
}
#endif

static enum speed_enum i2c_amd_get_bus_speed(struct platform_device *pdev)
{
	u32 acpi_speed;
	int i;
	static const u32 supported_speeds[] = {
		0, 100000, 400000, 1000000, 1400000, 3400000
	};

	acpi_speed = i2c_acpi_find_bus_speed(&pdev->dev);
	/* round down to the lowest standard speed */
	for (i = 1; i < ARRAY_SIZE(supported_speeds); i++) {
		if (acpi_speed < supported_speeds[i])
			break;
	}
	acpi_speed = supported_speeds[i - 1];

	switch (acpi_speed) {
	case 100000:
		return speed100k;
	case 400000:
		return speed400k;
	case 1000000:
		return speed1000k;
	case 1400000:
		return speed1400k;
	case 3400000:
		return speed3400k;
	default:
		return speed400k;
	}
}

static struct device *i2c_amd_acpi_get_first_phys_node(struct acpi_device *adev)
{
	const struct acpi_device_physical_node *node;

	if (list_empty(&adev->physical_node_list))
		return NULL;

	node = list_first_entry(&adev->physical_node_list,
				struct acpi_device_physical_node, node);
	return node->dev;
}

/*
 * On Lenovo Ideapad/Yoga the _DEP ACPI method appears to be the only available
 * hint at which PCI device an AMDI0011 ACPI device corresponds to.
 * i2c_amd_find_mp2_hint goes through the PCI devices enumerated by the _DEP
 * method and select the first listed MP2 device.
 */
static struct amd_mp2_dev *i2c_amd_find_mp2_hint(struct acpi_device *adev)
{
	struct acpi_handle_list dep_devices;
	acpi_status status;
	struct amd_mp2_dev *mp2_dev = NULL;
	int i;

	if (!acpi_has_method(adev->handle, "_DEP"))
		return NULL;

	status = acpi_evaluate_reference(adev->handle, "_DEP", NULL,
					 &dep_devices);
	if (ACPI_FAILURE(status))
		return NULL;

	for (i = 0; i < dep_devices.count; i++) {
		struct acpi_device *dep_adev;
		struct device *dep_phys_dev;
		struct pci_dev *pci_candidate;

		if (acpi_bus_get_device(dep_devices.handles[i], &dep_adev))
			continue;
		dep_phys_dev = i2c_amd_acpi_get_first_phys_node(dep_adev);

		if (!dep_phys_dev || !dev_is_pci(dep_phys_dev))
			continue;
		pci_candidate = to_pci_dev(dep_phys_dev);

		mp2_dev = amd_mp2_find_device(pci_candidate);
		if (mp2_dev)
			break;
	}

	return mp2_dev;
}

static const struct i2c_adapter_quirks amd_i2c_dev_quirks = {
	.max_read_len = AMD_MP2_I2C_MAX_RW_LENGTH,
	.max_write_len = AMD_MP2_I2C_MAX_RW_LENGTH,
};

static int i2c_amd_probe(struct platform_device *pdev)
{
	int ret;
	struct amd_i2c_dev *i2c_dev;
	acpi_handle handle = ACPI_HANDLE(&pdev->dev);
	struct acpi_device *adev;
	struct amd_mp2_dev *mp2_dev;
	const char *uid;

	if (acpi_bus_get_device(handle, &adev))
		return -ENODEV;

	mp2_dev = i2c_amd_find_mp2_hint(adev);
	if (!mp2_dev)
		/* If the hint pointed at a PCI device which isn't a MP2, go
		 * for the first MP2 device registered in the PCI driver
		 */
		mp2_dev = amd_mp2_find_device(NULL);
	if (!mp2_dev)
		/* The corresponding MP2 PCI device might get probed later */
		return -EPROBE_DEFER;

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->i2c_common.mp2_dev = mp2_dev;
	i2c_dev->pdev = pdev;
	platform_set_drvdata(pdev, i2c_dev);

	uid = adev->pnp.unique_id;
	if (!uid) {
		dev_err(&pdev->dev, "missing UID/bus id!\n");
		return -EINVAL;
	} else if (strcmp(uid, "0") == 0) {
		i2c_dev->i2c_common.bus_id = 0;
	} else if (strcmp(uid, "1") == 0) {
		i2c_dev->i2c_common.bus_id = 1;
	} else {
		dev_err(&pdev->dev, "incorrect UID/bus id \"%s\"!\n", uid);
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "bus id is %u\n", i2c_dev->i2c_common.bus_id);

	if (amd_mp2_register_cb(&i2c_dev->i2c_common))
		return -EINVAL;

	i2c_dev->i2c_common.i2c_speed = i2c_amd_get_bus_speed(pdev);

	/* setup i2c adapter description */
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.algo = &i2c_amd_algorithm;
	i2c_dev->adapter.quirks = &amd_i2c_dev_quirks;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.algo_data = i2c_dev;
	i2c_dev->adapter.nr = pdev->id;
	ACPI_COMPANION_SET(&i2c_dev->adapter.dev, ACPI_COMPANION(&pdev->dev));
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;
	snprintf(i2c_dev->adapter.name, sizeof(i2c_dev->adapter.name),
		 "AMD MP2 i2c bus %u", i2c_dev->i2c_common.bus_id);
	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);

	init_completion(&i2c_dev->msg_complete);

	/* enable the bus */
	amd_mp2_pm_runtime_get(mp2_dev);

	if (i2c_amd_xnable(i2c_dev, true))
		dev_err(&pdev->dev, "initial bus enable failed\n");

	amd_mp2_pm_runtime_put(mp2_dev);

	/* attach to the i2c layer */
	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);

	if (ret < 0)
		dev_err(&pdev->dev, "i2c add adapter failed = %d\n", ret);

	return ret;
}

void i2c_amd_delete_adapter(struct amd_i2c_common *i2c_common)
{
	struct amd_i2c_dev *i2c_dev = amd_i2c_dev_common(i2c_common);

	i2c_lock_bus(&i2c_dev->adapter, I2C_LOCK_ROOT_ADAPTER);
	if (!i2c_common->mp2_dev) {
		i2c_unlock_bus(&i2c_dev->adapter, I2C_LOCK_ROOT_ADAPTER);
		return;
	}

	i2c_amd_xnable(i2c_dev, false);

	amd_mp2_unregister_cb(i2c_common);

	i2c_common->mp2_dev = NULL;
	i2c_unlock_bus(&i2c_dev->adapter, I2C_LOCK_ROOT_ADAPTER);
	i2c_del_adapter(&i2c_dev->adapter);
}

static int i2c_amd_remove(struct platform_device *pdev)
{
	struct amd_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	struct amd_i2c_common *i2c_common = &i2c_dev->i2c_common;

	i2c_amd_delete_adapter(i2c_common);

	return 0;
}

static const struct acpi_device_id i2c_amd_acpi_match[] = {
	{ "AMDI0011" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, i2c_amd_acpi_match);

static struct platform_driver i2c_amd_plat_driver = {
	.probe = i2c_amd_probe,
	.remove = i2c_amd_remove,
	.driver = {
		.name = "i2c_amd_mp2",
		.acpi_match_table = ACPI_PTR(i2c_amd_acpi_match),
	},
};

int i2c_amd_register_driver(void)
{
	return platform_driver_register(&i2c_amd_plat_driver);
}

void i2c_amd_unregister_driver(void)
{
	platform_driver_unregister(&i2c_amd_plat_driver);
}
