## i2c-amd-mp2

This is a major rework of the proposed [MP2 I2C controller driver](https://patchwork.kernel.org/patch/10597369/) initially written by AMD for recent Ryzen-based laptops (e.g Dell Latitude 5495, Lenovo Yoga 530/Ideapad 530s, ...'s touchpad/touchscreen), which had a number of issues:

 * couldn't work with more than one bus
 * assumed that there's only one slave
 * ignored bus speeds specified by ACPI tables
 * buffer overreads/overflows when message length wasn't a multiple of 4
 * allocated heap/coherent DMA buffers for every read/write
 * no timeout checks, assumed the i2c device would respond in less than 50 jiffies
 * no protection against data races

This rewrite fixes all of these issues and was accepted into the 5.2 kernel.

### Installation

```bash
    sudo ./dkms-install.sh
```

:warning: **If your distribution is Ubuntu-based** :warning:, your kernel may ship with AMD's initial driver which is broken on some laptops and can't be overridden by a DKMS module, therefore before installing the driver you also need to install a kernel from http://kernel.ubuntu.com/~kernel-ppa/mainline/
