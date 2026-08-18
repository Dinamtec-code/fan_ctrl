#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tusb.h"
#include "tinyusb.h"

#include "usb_tmc_init.h"
#include "usb_tmc_process.h"
#include "esp_log.h"

static const char *TAG_USB = "USB_TMC_drv_init";

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV (0x10u)
#define IEEE4882_STB_SER (0x20u)
#define IEEE4882_STB_SRQ (0x40u)

#define BOARD_TUD_RHPORT 0

// 0=not query, 1=queried, 2=delay,set(MAV), 3=delay 4=ready?
// (to simulate delay)
static volatile uint16_t queryState = 0;
static volatile uint32_t queryDelayStart;
static volatile uint32_t bulkInStarted;
static volatile uint32_t idnQuery;
extern bool usb_host_connected;

void tmc_hal_init(void)
{
  ESP_LOGI(TAG_USB, "Inicializando USB-TMC con la estructura v6.0.2...");

  const tinyusb_config_t tusb_cfg = {
      .port = TINYUSB_PORT_FULL_SPEED_0, // Corregido: constante del enum

      .task = {
          .size = 4096, // Corregido: el campo se llama 'size'
          .priority = 5,
          .xCoreID = 1, // Obligatorio en tu versión
      },

      .descriptor = {
          .device = usb_desc_get_dev(),
          .qualifier = NULL,
          .string = usb_desc_get_string_desc(),
          .string_count = usb_desc_get_string_desc_count(),
          .full_speed_config = usb_desc_get_cfg(),
          .high_speed_config = NULL,
      },

      .phy = {
          .skip_setup = false,
          .self_powered = false,
          .vbus_monitor_io = -1,
      },

      .event_cb = NULL,
      .event_arg = NULL,
  };
  // driver_register(NULL);
  esp_err_t err = tinyusb_driver_install(&tusb_cfg);
  ESP_LOGI(TAG_USB, "Driver install retornó: %d", err);
  ESP_ERROR_CHECK(err);
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  // Nada que hacer en la etapa DATA o ACK
  if (stage != CONTROL_STAGE_SETUP)
    return true;

  // Si es la petición que definimos para MS OS 2.0 (Vendor Code = 0x01)
  if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x01)
  {
    if (request->wIndex == 7)
    {
      // El host pide el Descriptor de Features
      return tud_control_xfer(rhport, request, (void *)get_desc_ms_os_20_features(), get_desc_ms_os_20_f_count());
    }
  }

  // Si es otra cosa, lo rechazamos (Stall)
  return false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  usb_host_connected = false; // Tratar suspensión como "no activo"
  ESP_LOGI(TAG_USB, "USB Despertado");
}

void tud_resume_cb(void)
{ // Solo se reactiva si estaba montado antes de suspender
  usb_host_connected = tud_mounted();
  ESP_LOGI(TAG_USB, "USB resumen");
}

bool tud_get_connection(void)
{
  return usb_host_connected;
}
