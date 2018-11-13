## i2c-amd-mp2

This is a major rework of the proposed [MP2 I2C controller driver](https://patchwork.kernel.org/patch/10597369/) initially written by AMD for recent Ryzen-based laptops (e.g Dell Latitude 5495, Lenovo Yoga 530/Ideapad 530s, ...'s touchpad/touchscreen), which had a number of issues:

 * couldn't work with more than one bus
 * assumed that there's only one slave
 * ignored bus speeds specified by ACPI tables
 * buffer overreads/overflows when message length wasn't a multiple of 4
 * allocated heap/coherent DMA buffers for every read/write
 * no timeout checks, assumed the i2c device would respond in less than 50 jiffies
 * no protection against data races

This rewrite fixes all of these issues and is being submitted to the kernel.

### Installation

```bash
    sudo ./dkms-install.sh
```
