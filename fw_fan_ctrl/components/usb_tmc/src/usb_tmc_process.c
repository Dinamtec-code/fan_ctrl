#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "tusb.h"
#include "class/usbtmc/usbtmc_device.h"
#include "tinyusb.h"

#include "scpi/scpi.h"

#include "usb_tmc_cb.h"
#include "usb_tmc_process.h"
#include "usb_tmc_init.h"

const static char *TAG = "tmc_fsm_task";

/**********************************************************************************************************
 * Buffers locales y coneccion directa con el scpi
 *
 *********************************************************************************************************/
#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024 * 16

// static uint8_t rx_buffer_odd[RX_BUFFER_SIZE] = {0};
// static uint8_t rx_buffer_even[RX_BUFFER_SIZE] = {0};
static uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};
static size_t data_2_tx = 0;

static scpi_t scpi_context;
usb_tmc_status_t usb_tmc_status_reg;

usb_tmc_state_t tmc_state;

bool tmc_scpi_has_errors(void)
{
    return (bool)(scpi_context.error_queue.count > 0);
}

bool tmc_scpi_has_connected(void)
{
    return tud_get_connectoin();
}

/**********************************************************************************************************
 * SCPI
 *
 *********************************************************************************************************/

float kp[4] = {1.0f, 2.0f, 3.0f, 4.0f};
float ki[4] = {5.0f, 6.0f, 7.0f, 8.0f};
float kd[4] = {9.0f, 10.0f, 11.0f, 12.0f};

float temp[2] = {25.0f, 26.0f};
float set_point[2] = {35.0f, 36.0f};

float duty[2] = {0.5f, 0.5f};

bool output_status[2] = {0, 0};

/* ==========================================================================
 * Contexto global de libscpi y buffers asociados
 * ========================================================================== */

#define SCPI_INPUT_BUFFER_LENGTH 1024
static char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];

#define SCPI_ERROR_QUEUE_SIZE 32
static scpi_error_t scpi_error_queue[SCPI_ERROR_QUEUE_SIZE];

static scpi_result_t my_CoreRst(scpi_t *context);

/* ==========================================================================
 * Funciones auxiliares de los comandos del instrumento
 * ========================================================================== */

static float get_temperature(void)
{
    return temp[0];
}

static float get_setpoint(uint8_t channel)
{
    return set_point[channel - 1];
}

static void set_setpoint(uint8_t channel, float val)
{
    set_point[channel - 1] = val;
}

static float get_kp(void)
{
    return kp[0];
}

static void set_kp(float val)
{
    kp[0] = val;
}

static float get_ki(void)
{
    return ki[0];
}

static void set_ki(float val)
{
    ki[0] = val;
}

static float get_kd(void)
{
    return kd[0];
}

static void set_kd(float val)
{
    kd[0] = val;
}

static float get_duty(void)
{
    return duty[0];
}

static bool get_output(uint8_t channel)
{
    return output_status[channel - 1];
}

static void set_output(uint8_t channel, bool on)
{
    output_status[channel - 1] = on;
}

/* ==========================================================================
 * Callbacks de comandos SCPI
 * ========================================================================== */

static scpi_result_t
cmd_meas_temp(scpi_t *context)
{

    SCPI_ResultFloat(context, get_temperature());
    return SCPI_RES_OK;
}

static scpi_result_t cmd_temp_sp_q(scpi_t *context)
{
    int32_t numbers[1]; // Arreglo para capturar los sufijos del comando

    // 1. Obtener el número de canal del nodo raíz (ej. SOURce1 -> 1)
    // El último parámetro es el valor por defecto si el usuario envía "SOUR:TEMP..." sin número.
    SCPI_CommandNumbers(context, numbers, 1, 1);
    int32_t channel = numbers[0];

    // 2. Validar que el canal exista en tu hardware
    if (channel < 1 || channel > 2)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultFloat(context, get_setpoint(channel));
    return SCPI_RES_OK;
}

scpi_result_t cmd_temp_sp(scpi_t *context)
{
    int32_t numbers[1]; // Arreglo para capturar los sufijos del comando
    float setpoint;

    // 1. Obtener el número de canal del nodo raíz (ej. SOURce1 -> 1)
    // El último parámetro es el valor por defecto si el usuario envía "SOUR:TEMP..." sin número.
    SCPI_CommandNumbers(context, numbers, 1, 1);
    int32_t channel = numbers[0];

    // 2. Validar que el canal exista en tu hardware
    if (channel < 1 || channel > 2)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    // 3. Extraer el parámetro enviado por el usuario
    if (!SCPI_ParamFloat(context, &setpoint, TRUE))
    {
        return SCPI_RES_ERR;
    }

    set_setpoint(channel, setpoint);

    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_kp_q(scpi_t *context)
{
    SCPI_ResultFloat(context, get_kp());
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_kp(scpi_t *context)
{
    float val;
    if (SCPI_ParamFloat(context, &val, true))
    {
        set_kp(val);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}

static scpi_result_t cmd_pid_ki_q(scpi_t *context)
{
    SCPI_ResultFloat(context, get_ki());
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_ki(scpi_t *context)
{
    float val;
    if (SCPI_ParamFloat(context, &val, true))
    {
        set_ki(val);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}

static scpi_result_t cmd_pid_kd_q(scpi_t *context)
{
    SCPI_ResultFloat(context, get_kd());
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_kd(scpi_t *context)
{
    float val;
    if (SCPI_ParamFloat(context, &val, true))
    {
        set_kd(val);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}

static scpi_result_t cmd_pid_duty_q(scpi_t *context)
{
    SCPI_ResultFloat(context, get_duty());
    return SCPI_RES_OK;
}

static scpi_result_t cmd_sour_outp(scpi_t *context)
{
    uint64_t channel;
    scpi_bool_t on;
    if (!SCPI_ParamUInt64(context, &channel, true))
        return SCPI_RES_ERR;
    if (channel > 1)
        return SCPI_RES_ERR;
    if (!SCPI_ParamBool(context, &on, true))
        return SCPI_RES_ERR;
    set_output((uint8_t)channel, on);
    return SCPI_RES_OK;
}

static scpi_result_t cmd_sour_outp_q(scpi_t *context)
{
    uint64_t channel;
    if (!SCPI_ParamUInt64(context, &channel, true))
        return SCPI_RES_ERR;
    if (channel > 1)
        return SCPI_RES_ERR;
    SCPI_ResultBool(context, get_output((uint8_t)channel));
    return SCPI_RES_OK;
}

/**
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreTstQ(scpi_t *context)
{

    SCPI_ResultInt32(context, 0);

    return SCPI_RES_OK;
}

/* ==========================================================================
 * Árbol de comandos del instrumento
 * ========================================================================== */

static const scpi_command_t scpi_commands[] = {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    {
        .pattern = "*CLS",
        .callback = SCPI_CoreCls,
    },
    {
        .pattern = "*ESE",
        .callback = SCPI_CoreEse,
    },
    {
        .pattern = "*ESE?",
        .callback = SCPI_CoreEseQ,
    },
    {
        .pattern = "*ESR?",
        .callback = SCPI_CoreEsrQ,
    },
    {
        .pattern = "*IDN?",
        .callback = SCPI_CoreIdnQ,
    },
    {
        .pattern = "*OPC",
        .callback = SCPI_CoreOpc,
    },
    {
        .pattern = "*OPC?",
        .callback = SCPI_CoreOpcQ,
    },
    {
        .pattern = "*RST",
        .callback = my_CoreRst,
        //        .callback = SCPI_CoreRst,
    },
    {
        .pattern = "*SRE",
        .callback = SCPI_CoreSre,
    },
    {
        .pattern = "*SRE?",
        .callback = SCPI_CoreSreQ,
    },
    {
        .pattern = "*STB?",
        .callback = SCPI_CoreStbQ,
    },
    {
        .pattern = "*TST?",
        .callback = My_CoreTstQ,
    },
    {
        .pattern = "*WAI",
        .callback = SCPI_CoreWai,
    },

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {
        .pattern = "SYSTem:ERRor[:NEXT]?",
        .callback = SCPI_SystemErrorNextQ,
    },
    {
        .pattern = "SYSTem:ERRor:COUNt?",
        .callback = SCPI_SystemErrorCountQ,
    },
    {
        .pattern = "SYSTem:VERSion?",
        .callback = SCPI_SystemVersionQ,
    },
    /* {.pattern = "STATus:OPERation?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:EVENt?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:CONDition?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle?", .callback = scpi_stub_callback,}, */
    {
        .pattern = "STATus:QUEStionable[:EVENt]?",
        .callback = SCPI_StatusQuestionableEventQ,
    },
    /* {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,}, */
    {
        .pattern = "STATus:QUEStionable:ENABle",
        .callback = SCPI_StatusQuestionableEnable,
    },
    {
        .pattern = "STATus:QUEStionable:ENABle?",
        .callback = SCPI_StatusQuestionableEnableQ,
    },

    {
        .pattern = "STATus:PRESet",
        .callback = SCPI_StatusPreset,
    },
    /* Medición */
    {.pattern = "MEASure:TEMPerature#?", .callback = cmd_meas_temp, .tag = 0},

    /* Setpoint */
    {.pattern = "SOURce#:TEMPerature:SETPOint?", .callback = cmd_temp_sp_q, .tag = 0},
    {.pattern = "SOURce#:TEMPerature:SETPOint", .callback = cmd_temp_sp, .tag = 0},

    /* PID */
    {.pattern = "SOURce#:CONTrol:PID:KP?", .callback = cmd_pid_kp_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KP", .callback = cmd_pid_kp, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KI?", .callback = cmd_pid_ki_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KI", .callback = cmd_pid_ki, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KD?", .callback = cmd_pid_kd_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KD", .callback = cmd_pid_kd, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:OUTPut?", .callback = cmd_pid_duty_q, .tag = 0},

    /* Salidas */
    {.pattern = "SOURce#:OUTPut", .callback = cmd_sour_outp, .tag = 0},
    {.pattern = "SOURce#:OUTPut?", .callback = cmd_sour_outp_q, .tag = 0},

    SCPI_CMD_LIST_END};

/* ==========================================================================
 * Callbacks de la interfaz de libscpi
 * ========================================================================== */
usb_tmc_status_t usb_tmc_get_stb(void)
{
    return usb_tmc_status_reg;
}

static scpi_result_t my_CoreRst(scpi_t *context)
{
    (void)context;
    SCPI_CoreRst(context); // Reset del parser y registros IEEE 488.2
    set_point[0] = 0.0f;
    set_point[1] = 0.0f;

    kp[0] = 1.0f;
    kp[1] = 2.0f;
    ki[0] = 5.0f;
    ki[1] = 6.0f;
    kd[0] = 9.0f;
    kd[1] = 10.0f;

    temp[0] = 25.0f;
    temp[1] = 26.0f;
    set_point[0] = 35.0f;
    set_point[1] = 36.0f;

    duty[0] = 0.5f;
    duty[1] = 0.5f;

    output_status[0] = 0;
    output_status[1] = 0;
    // config_apply_reset_values(); // Restauración de los parámetros del instrumento
    return SCPI_RES_OK;
}

static int scpi_error_cb(scpi_t *ctx, int_fast16_t err)
{
    (void)ctx;
    (void)err;
    return SCPI_RES_OK;
}

static size_t scpi_write_cb(scpi_t *ctx, const char *data, size_t len)
{
    (void)ctx;

    if (len + data_2_tx < TX_BUFFER_SIZE)
    {
        memcpy(&(tx_buffer[data_2_tx]), data, len);
        data_2_tx += len;
    }
    else
    {
        scpi_error_cb(&scpi_context, 250);
    }
    return len;
}

static scpi_result_t scpi_reset_cb(scpi_t *ctx)
{
    (void)ctx;
    return SCPI_RES_OK;
}

static scpi_result_t scpi_control_cb(scpi_t *ctx, scpi_ctrl_name_t ctrl, scpi_reg_val_t val)
{
    (void)ctx;
    (void)val;
    (void)ctrl;
    return SCPI_RES_OK;
}

static scpi_result_t scpi_flush_cb(scpi_t *ctx)
{
    (void)ctx;
    return SCPI_RES_OK;
}

static scpi_interface_t scpi_interface = {
    .error = scpi_error_cb,
    .write = scpi_write_cb,
    .control = scpi_control_cb,
    .flush = scpi_flush_cb,
    .reset = scpi_reset_cb,
};

/* ==========================================================================
 * API pública del parser SCPI
 * ========================================================================== */

void scpi_engine_init(void)
{
    SCPI_Init(&scpi_context,
              scpi_commands,
              &scpi_interface,
              scpi_units_def,
              "Dinamtec",
              "Controlador de Ventilador",
              "SCV",
              "v1.0.1",
              scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
              scpi_error_queue, SCPI_ERROR_QUEUE_SIZE);
}

/**********************************************************************************************************
 * State machin
 *
 *********************************************************************************************************/

static inline void clear_tx_buffer()
{
    memset(tx_buffer, 0, TX_BUFFER_SIZE);
    data_2_tx = 0;
}
//
void usb_tmc_fsm_process(usb_tmc_event_t event, void *data, size_t len)
{
    switch (tmc_state)
    {
    case STATE_TMC_IDLE:
        if (event == EV_TMC_RX_START)
        {
            ESP_LOGW(TAG, "New massage");
            tmc_state = STATE_TMC_RECEIVING;
        }
        else if (event == EV_TMC_TX_REQ)
        {
            scpi_error_cb(&scpi_context, 420);
            ESP_LOGI(TAG, "Se publico y limpio el error MSG_ERR_UNTERMIN");
            clear_tx_buffer();
            tud_usbtmc_transmit_dev_msg_data(NULL, 0, true, false); // NAK/Empty
        }
        break;
    case STATE_TMC_RECEIVING:
        SCPI_Input(&scpi_context, data, len);
        ESP_LOGI(TAG, "Se recivieron %d datos", (int)len);
        if (event == EV_TMC_RX_END)
        {
            ESP_LOGI(TAG, "Recepción Completa ");
            if (data_2_tx > 0)
            {
                tmc_state = STATE_TMC_REPLY_READY;
            }
            else
            {
                data_2_tx = 0;
                tmc_state = STATE_TMC_IDLE;
            }
            // Le decimos a TinyUSB que estamos listos para recibir comandos nuevos
            tud_usbtmc_start_bus_read();
        }
        else if (event == EV_TMC_RX_CHUNK)
        {
            // Acción: Guardar fragmento sin bloquear
        }
        break;
    case STATE_TMC_PROCESSING:
        break;
    case STATE_TMC_REPLY_READY:
        if (event == EV_TMC_TX_REQ)
        { // Verificamos si este es el ÚLTIMO fragmento del mensaje
            bool eof = (tx_buffer[data_2_tx - 1] == '\n');
            data_2_tx = tu_min32(data_2_tx, len); // Respetamos lo que el Host nos pidió leer
            // Transmitimos. Si eof == true, TinyUSB asertará el bit EOM en el header
            tud_usbtmc_transmit_dev_msg_data((const void *)tx_buffer, data_2_tx, eof, false);
            ESP_LOGI(TAG, "Enviando respuesta");
        }
        else if (event == EV_TMC_TX_DONE)
        {
            clear_tx_buffer();
            usb_tmc_status_reg &= ~STB_MAV_BIT; // ¡MAV se apaga al terminar!
            tmc_state = STATE_TMC_IDLE;
            ESP_LOGI(TAG, "Respuesta enviada");
        }
        else if (event == EV_TMC_RX_START)
        {
            scpi_error_cb(&scpi_context, 410);
            ESP_LOGI(TAG, "Se publico el error MSG_ERR_INTERRUPT");
            clear_tx_buffer();
            tmc_state = STATE_TMC_RECEIVING;
            ESP_LOGW(TAG, "New massage");
        }
        break;

    default:
        ESP_LOGI(TAG, "Estado usb tmb desconocido...");
    }
}
