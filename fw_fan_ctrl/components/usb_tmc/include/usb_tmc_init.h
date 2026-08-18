#ifndef USB_TMC_INIT_H_
#define USB_TMC_INIT_H_

#include <stdint.h>
#include <stddef.h>

#include "tinyusb.h"
#include "tusb.h"

#ifdef __cpluslpus
extern "C"
{
#endif

    void tmc_hal_init(void);
    void usbtmc_app_task(void *pvParameters);

    tusb_desc_device_t const *usb_desc_get_dev(void);
    uint8_t const *usb_desc_get_cfg(void);
    char const **usb_desc_get_string_desc(void);
    int usb_desc_get_string_desc_count(void);
    uint8_t const *get_desc_ms_os_20_features(void);
    size_t get_desc_ms_os_20_f_count(void);

#ifdef __cplusplus
}
#endif

#endif
