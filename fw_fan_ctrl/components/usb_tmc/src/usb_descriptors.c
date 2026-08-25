#include <stdint.h>
#include <stddef.h>

#include "usb_tmc_config.h"
#include "usb_tmc_init.h"

#include "device/usbd.h"
#include "class/usbtmc/usbtmc.h"
#include "class/usbtmc/usbtmc_device.h"

// ============================================================================
// CONFIGURACIÓN DE IDENTIFICADORES (Modificable según tu hardware/empresa)
// ============================================================================
#define USB_VID 0x16C0 // V-USB/libus;  // 0x03EB // Ejemplo (Atmel/Microchip o tu propio VID)
#define USB_PID 0x2044 // PID asignado a instrumentación TMC
#define USB_BCD 0x0100 // Versión del firmware 1.00

// Configuración de la interfaz TMC
#define EPNUM_TMC_BULK_OUT 0x01
#define EPNUM_TMC_BULK_IN 0x81
#define EPNUM_TMC_INT_IN 0x82

// Tamaños de los endpoints (64 bytes es el máximo para Bulk en Full-Speed del ESP32-S3)
#define TMC_BULK_EP_SIZE 64
#define TMC_INT_EP_SIZE 8

// Tamaño real sumando los componentes de tu usbd.h (9 + 14 + 7 = 30 bytes)
#define TUD_TMC_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_USBTMC_IF_DESCRIPTOR_LEN + TUD_USBTMC_BULK_DESCRIPTORS_LEN + TUD_USBTMC_INT_DESCRIPTOR_LEN)
#define DESC_BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

// Definiciones manuales si los macros no existen
#define TUD_USBTMC_CLASS 0xFE     // Application Specific
#define TUD_USBTMC_SUBCLASS 0x03  // USBTMC
#define TUD_USBTMC_PROTO_STD 0x00 // USBTMC
#define TUD_USBTMC_PROTO_488 0x01 // USB488

// ============================================================================
// 1. DEVICE DESCRIPTOR
// ============================================================================
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    // .bDeviceClass = TUD_USBTMC_CLASS,        // 0x01 para descriptor de de tipo dispositivo
    // .bDeviceSubClass = TUD_USBTMC_SUBCLASS,  // 0xFE
    // .bDeviceProtocol = TUD_USBTMC_PROTO_488, // 0x03

    .bDeviceClass = 0x00,                      // (CORREGIDO) Definido a nivel de interfaz
    .bDeviceSubClass = 0x00,                   // (CORREGIDO)
    .bDeviceProtocol = 0x00,                   // (CORREGIDO)
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, // 64 bytes para maximo tamaño de paquete de control

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD,

    .iManufacturer = 0x01, // Índice del string del fabricante
    .iProduct = 0x02,      // Índice del string del producto
    .iSerialNumber = 0x03, // Índice del string del número de serie
    .bNumConfigurations = 0x01};

// ============================================================================
// 2. CONFIGURATION DESCRIPTOR (TMC Interface)
// ============================================================================
uint8_t const desc_configuration[] = {
    // Configuración: 1 interfaz, longitud total correcta, autoalimentado, 100 mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_TMC_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interfaz USBTMC (protocolo USB488)
    TUD_USBTMC_IF_DESCRIPTOR(0, 3, 0, TUD_USBTMC_PROTOCOL_USB488),

    // Endpoints Bulk OUT e IN
    TUD_USBTMC_BULK_DESCRIPTORS(0x01, 0x81, TMC_BULK_EP_SIZE),

    // Endpoint de interrupción IN (obligatorio para USB488)
    TUD_USBTMC_INT_DESCRIPTOR(0x82, TMC_INT_EP_SIZE, 10), // bInterval = 10 ms
};

// BOS Descriptor
uint8_t const desc_bos[] = {
    // Total length, number of device caps
    TUD_BOS_DESCRIPTOR(DESC_BOS_TOTAL_LEN, 1),
    // Microsoft OS 2.0 descriptor (Vendor Code = 0x01)
    TUD_BOS_MS_OS_20_DESCRIPTOR(DESC_BOS_TOTAL_LEN, 0x01)};

// Callback débil de TinyUSB para devolver el BOS
uint8_t const *tud_descriptor_bos_cb(void)
{
  return NULL;
  //  return desc_bos;
}

uint8_t const desc_ms_os_20_features[] = {
    // Set header: Length, Type, Windows version, Total Length
    U16_TO_U8S_LE(0x000A),
    U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000),
    U16_TO_U8S_LE(0x00B4),

    // Configuration subset header
    U16_TO_U8S_LE(0x0008),
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0,
    0,
    U16_TO_U8S_LE(0x00AA),

    // Function Subset header
    U16_TO_U8S_LE(0x0008),
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    0,
    0,
    U16_TO_U8S_LE(0x00A2),

    // MS OS 2.0 Compatible ID descriptor: "WINUSB"
    U16_TO_U8S_LE(0x0014),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W',
    'I',
    'N',
    'U',
    'S',
    'B',
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

// ============================================================================
// 3. STRING DESCRIPTORS (Los nombres que verá Windows y NI-VISA)
// ============================================================================
static char const *string_desc_arr[] = {
    "\x09\x04",             // 0: Idioma (0x0409 = Inglés US)
    "Dinamtec Electronica", // 1: Fabricante
    "Control de velocidad", // 2: Producto
    "DTSCV1S01",            // 3: Número de Serie
    NULL};

tusb_desc_device_t const *usb_desc_get_dev()
{
  return &desc_device;
}

uint8_t const *usb_desc_get_cfg()
{
  return desc_configuration;
}

char const **usb_desc_get_string_desc()
{
  return (const char **)string_desc_arr;
}

int usb_desc_get_string_desc_count()
{
  return (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) - 1;
}

uint8_t const *get_desc_ms_os_20_features()
{
  return desc_ms_os_20_features;
}

size_t get_desc_ms_os_20_f_count()
{
  return (size_t)(sizeof(desc_ms_os_20_features));
}