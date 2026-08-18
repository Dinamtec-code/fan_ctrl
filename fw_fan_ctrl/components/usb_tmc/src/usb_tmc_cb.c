#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "tusb.h"
#include "tinyusb.h"
#include "class/usbtmc/usbtmc_device.h"
#include "esp_log.h"

#include "usb_tmc_process.h"

static const char *TAG_TMC = "usb_tmc_cb";
static volatile uint8_t status;
bool usb_host_connected = false;

// 1. Implementación de Capabilities (Requerido)
static usbtmc_response_capabilities_488_t tud_usbtmc_app_capabilities = {
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities =
        {
            .listenOnly = 0,
            .talkOnly = 0,
            .supportsIndicatorPulse = 1},
    .bmDevCapabilities = {
        .canEndBulkInOnTermChar = 0},
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 = {.supportsTrigger = 1, .supportsREN_GTL_LLO = 0, .is488_2 = 1},
    .bmDevCapabilities488 = {.SCPI = 1, .SR1 = 0, .RL1 = 0, .DT1 = 0}};

usbtmc_response_capabilities_488_t const *tud_usbtmc_get_capabilities_cb(void)
{
    return &tud_usbtmc_app_capabilities;
}

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV (0x10u)
#define IEEE4882_STB_SER (0x20u)
#define IEEE4882_STB_SRQ (0x40u)

// 2. Callback de apertura (Requerido para inicializar el bus)
void tud_usbtmc_open_cb(uint8_t interface_id)
{
    ESP_LOGI(TAG_TMC, "Interface USBTMC abierta (ID: %d)", interface_id);
    tud_usbtmc_start_bus_read();
}

// Variable en el contexto de tu driver para recordar si este mensaje trae EOM
static bool current_msg_has_eom = false;

// 3. Callback de recepción de mensajes BULK
bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const *msgHeader)
{
    ESP_LOGI(TAG_TMC, "Mensaje bulk out start");
    usb_host_connected = tud_mounted();
    // El estándar USB-TMC define que el Bit 0 de bmTransferAttributes es el flag EOM
    current_msg_has_eom = msgHeader->bmTransferAttributes.EOM != 0;

    uint32_t transfer_size = msgHeader->TransferSize;
    ESP_LOGI(TAG_TMC, "transferSize: %lu, EOM: %d", transfer_size, (int)current_msg_has_eom);

    // Avisamos a la FSM que empezamos a recibir datos (Bulk OUT)
    usb_tmc_fsm_process(EV_TMC_RX_START, NULL, 0);
    return true;
}

// 4. Callback de datos recibidos
bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete)
{
    ESP_LOGI(TAG_TMC, "Datos (len: %d)", len);

    if (transfer_complete)
    {
        // 1. Primero metemos el último pedazo de datos tal cual vino
        usb_tmc_fsm_process(EV_TMC_RX_CHUNK, data, len);

        // 2. Revisamos si el host asertó EOM por hardware pero olvidó el '\n'
        if (current_msg_has_eom && len > 0)
        {
            char last_char = ((char *)data)[len - 1];
            // Si no termina en \n (ni \r por si acaso), lo inyectamos
            if (last_char != '\n' && last_char != '\r')
            {
                char nl = '\n';
                // Usamos el mismo evento CHUNK para inyectar este byte extra
                usb_tmc_fsm_process(EV_TMC_RX_CHUNK, &nl, 1);
            }
        }

        // 3. Le avisamos a la FSM que el mensaje USB terminó completamente.
        // La FSM pasará a STATE_TMC_PROCESSING y despertará a libSCPI.
        usb_tmc_fsm_process(EV_TMC_RX_END, NULL, 0);
    }
    else
    {
        // Si no está completo, solo pasamos los datos
        usb_tmc_fsm_process(EV_TMC_RX_CHUNK, data, len);
    }

    tud_usbtmc_start_bus_read();
    return true;
}

// 5. Callback de petición BULK IN (El host pide datos)
bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const *request)
{
    ESP_LOGI(TAG_TMC, "Host pide datos (Bulk IN)");
    static size_t msgReqLen;

    msgReqLen = (size_t)(request->TransferSize);
    usb_host_connected = tud_mounted();
    usb_tmc_fsm_process(EV_TMC_TX_REQ, NULL, msgReqLen);
    ESP_LOGI(TAG_TMC, "mensaje bulk request");
    return true;
}

// 6. Callback de finalización BULK IN*
bool tud_usbtmc_msgBulkIn_complete_cb(void)
{
    ESP_LOGI(TAG_TMC, "mensaje bulk in complete");
    usb_tmc_fsm_process(EV_TMC_TX_DONE, NULL, 0);
    tud_usbtmc_start_bus_read();
    return true;
}

// 7. Callback para el STATUS BYTE (IEEE 488) - CRITICO PARA SCPI
uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult)
{
    ESP_LOGI(TAG_TMC, "Get Status Byte ");
    *tmcResult = USBTMC_STATUS_SUCCESS;
    return (uint8_t)(usb_tmc_get_stb() & STB_ESB_BIT);
}

// 8. Callback para el TRIGGER (Ejemplo: comando *TRG o señal GET)
bool tud_usbtmc_msg_trigger_cb(usbtmc_msg_generic_t *msg)
{
    ESP_LOGI(TAG_TMC, "Trigger recibido!");
    // Aquí conectaremos la función de Trigger de SCPI más adelante
    return true;
}

// 9. Clear Feature Bulk OUT
void tud_usbtmc_bulkOut_clearFeature_cb(void)
{
    ESP_LOGI(TAG_TMC, "Bulk out clear Feature");
    // Normalmente se rearma la lectura aquí
    tud_usbtmc_start_bus_read();
}

// 10. Clear Feature Bulk IN
void tud_usbtmc_bulkIn_clearFeature_cb(void)
{
    ESP_LOGI(TAG_TMC, "Bulk in clear Featute ");
    // Nada especial por ahora
}

// 11. Initiate Abort Bulk OUT
bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult)
{
    ESP_LOGI(TAG_TMC, "Initiate abort bulk out");
    *tmcResult = USBTMC_STATUS_SUCCESS;
    return true;
}

// 12. Check Abort Bulk OUT
bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
    ESP_LOGI(TAG_TMC, "Check abort bulk out");
    rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
    rsp->bmAbortBulkIn.BulkInFifoBytes = 0;
    tud_usbtmc_start_bus_read();
    return true;
}

// 13. Initiate Abort Bulk IN
bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult)
{
    ESP_LOGI(TAG_TMC, "Initiate abort bulk in");
    *tmcResult = USBTMC_STATUS_SUCCESS;
    return true;
}

// 14. Check Abort Bulk IN
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
    ESP_LOGI(TAG_TMC, "Check abort bulk in");
    rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
    rsp->bmAbortBulkIn.BulkInFifoBytes = 0;
    tud_usbtmc_start_bus_read();
    return true;
}

// 15. Initiate Clear
bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult)
{
    ESP_LOGI(TAG_TMC, "Init clear");
    *tmcResult = USBTMC_STATUS_SUCCESS;
    return true;
}

// 16. Check Clear
bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp)
{
    ESP_LOGI(TAG_TMC, "Check clear");
    rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
    rsp->bmClear.BulkInFifoBytes = 0;
    tud_usbtmc_start_bus_read();
    return true;
}
