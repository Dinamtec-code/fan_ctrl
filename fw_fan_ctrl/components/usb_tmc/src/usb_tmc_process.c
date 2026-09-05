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

#include "ctrl_data.h"
#include "ctrl_output.h"

#include "adq_cfg.h"

const static char *TAG = "tmc_fsm_task";

/**********************************************************************************************************
 * Buffers locales y coneccion directa con el scpi
 *
 *********************************************************************************************************/
#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024 * 16

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
    return tud_get_connection();
}

/******************************************************************************
 * SCPI
 ******************************************************************************/

static inline scpi_result_t get_root_channel(scpi_t *context, int32_t *channel)
{
    SCPI_CommandNumbers(context, channel, 1, 1);
    *channel = *channel - 1;
    if (*channel < 0 || *channel >= CTRL_NUM_CHANNELS)
    {
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

/******************************************************************************
 * Contexto global de libscpi y buffers asociados
 ******************************************************************************/
typedef enum
{
    FORMAT_ASCII,
    FORMAT_BINARY

} data_format_t;

static data_format_t data_format[CTRL_NUM_CHANNELS] = {FORMAT_ASCII, FORMAT_ASCII};

#define SCPI_INPUT_BUFFER_LENGTH 1024
static char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];

#define SCPI_ERROR_QUEUE_SIZE 32
static scpi_error_t scpi_error_queue[SCPI_ERROR_QUEUE_SIZE];

/* ==========================================================================
 * Callbacks de la interfaz de libscpi
 * ==========================================================================*/
// usb_tmc_status_t usb_tmc_get_stb(void)
// {
//     return usb_tmc_status_reg;
// }

uint8_t usb_tmc_get_stb(void)
{
    return (uint8_t)SCPI_RegGet(&scpi_context, SCPI_REG_STB);
}

static scpi_result_t custom_SCPI_CoreCls(scpi_t *context)
{
    SCPI_CoreCls(context);
    /**
     * TODO: implementar limpieza completa de buffers (sensor, control, adq, etc...)
     */
    return SCPI_RES_OK;
}

static scpi_result_t custom_CoreRst(scpi_t *context)
{
    (void)context;
    SCPI_CoreRst(context);

    for (uint8_t i = 0; i < CTRL_NUM_CHANNELS; i++)
    {
        /**
         * TODO: Implementar reinicio del hardware por software
         */
        ctrl_set_setpoint_value(i, 0.0f);
        ctrl_set_kp_value(i, 1.0f); // O el valor default que quieras
        ctrl_set_ki_value(i, 0.0f);
        ctrl_set_kd_value(i, 0.0f);
        ctrl_set_duty_value(i, 0.0f);
        ctrl_set_output_state(i, false);
        ctrl_set_controller_type(i, CTRL_OPEN);
    }
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

/******************************************************************************
 * Callbacks de comandos SCPI
 ******************************************************************************/
static scpi_result_t cmd_meas_speed_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultFloat(context, ctrl_get_sensor_speed((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_meas_data_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    ctrl_buffer_data_t *buffer = ctrl_get_buffers((uint8_t)channel);
    if (buffer == NULL)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    ctrl_sensor_data_t dato;

    if (!buffer->get_latest(buffer, &dato))
    {
        SCPI_ErrorPush(context, SCPI_ERROR_BLOCK_DATA_NOT_ALLOWED);
        return SCPI_RES_ERR;
    }

    if (data_format[channel] == FORMAT_ASCII)
    {
        SCPI_ResultFloat(context, dato.speed);
        SCPI_ResultUInt64(context, dato.timestamp_us);
    }
    else if (data_format[channel] == FORMAT_BINARY)
    {
        SCPI_ResultArbitraryBlock(context, &dato, sizeof(dato));
    }

    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_init(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    adq_adq_initialize(channel);
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_abort(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    adq_stop(channel);
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_fetch_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_count_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    SCPI_ResultInt64(context, adq_get_count(channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_cls(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    adq_data_clear(channel);
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_format_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    switch (data_format[channel])
    {
    case FORMAT_ASCII:
        SCPI_ResultCharacters(context, "ASCII", 5);
        break;
    case FORMAT_BINARY:
        SCPI_ResultCharacters(context, "BINARY", 6);
        break;
    default:
        SCPI_ResultCharacters(context, "Format error", 12);
        break;
    }
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_format(scpi_t *context)
{
    int32_t channel;
    const char *param_ptr;
    size_t param_len;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamCharacters(context, &param_ptr, &param_len, TRUE))
    {
        return SCPI_RES_ERR;
    }

    if (strncmp(param_ptr, "BINARY", param_len) == 0 && param_len == 6)
    {
        data_format[channel] = FORMAT_BINARY;
    }
    else if (strncmp(param_ptr, "ASCII", param_len) == 0 && param_len == 5)
    {
        data_format[channel] = FORMAT_ASCII;
    }

    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_pre_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    SCPI_ResultFloat(context, ctrl_get_adq_pretrigger_value((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_pre(scpi_t *context)
{
    int32_t channel;
    uint32_t pre_trigger;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamUInt32(context, &pre_trigger, TRUE))
    {
        return SCPI_RES_ERR;
    }

    ctrl_set_adq_pretrigger_value((uint8_t)channel, pre_trigger);

    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_pts(scpi_t *context)
{
    int32_t channel;
    uint32_t points;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamUInt32(context, &points, TRUE))
    {
        return SCPI_RES_ERR;
    }

    ctrl_set_adq_points_value((uint8_t)channel, points);
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_data_pts_q(scpi_t *context)
{
    int32_t channel;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultFloat(context, ctrl_get_adq_points_value((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_trig_cfg(scpi_t *context)
{
    const char *param_ptr;
    size_t param_len;
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamCharacters(context, &param_ptr, &param_len, TRUE))
    {
        return SCPI_RES_ERR;
    }

    if (strncmp(param_ptr, "SETPOINT", param_len) == 0 && param_len == 8)
    {
        ctrl_set_adq_trigger_source_value((uint8_t)channel, ADQ_TRIGGER_SETPOIT);
    }
    else if (strncmp(param_ptr, "OUTPUT", param_len) == 0 && param_len == 6)
    {
        ctrl_set_adq_trigger_source_value((uint8_t)channel, ADQ_TRIGGER_OUTPUT_ON);
    }
    else if (strncmp(param_ptr, "EXTERNAL", param_len) == 0 && param_len == 8)
    {
        ctrl_set_adq_trigger_source_value((uint8_t)channel, ADQ_TRIGGER_EXTERNAL);
    }
    else if (strncmp(param_ptr, "SOFTWARE", param_len) == 0 && param_len == 8)
    {
        ctrl_set_adq_trigger_source_value((uint8_t)channel, ADQ_TRIGGER_SOFTWARE);
    }
    return SCPI_RES_OK;
}

static scpi_result_t cmd_adq_trig_cfg_q(scpi_t *context)
{
    int32_t channel;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    switch (ctrl_get_adq_trigger_source_value((uint8_t)channel))
    {
    case ADQ_TRIGGER_SETPOIT:
        SCPI_ResultCharacters(context, "SetPoit", 7);
        break;
    case ADQ_TRIGGER_OUTPUT_ON:
        SCPI_ResultCharacters(context, "Ouput ON", 8);
        break;
    case ADQ_TRIGGER_EXTERNAL:
        SCPI_ResultCharacters(context, "External", 8);
        break;
    case ADQ_TRIGGER_SOFTWARE:
        SCPI_ResultCharacters(context, "Software", 8);
        break;
    default:
        SCPI_ResultCharacters(context, "ErrorCfg", 8);
        break;
    }

    return SCPI_RES_OK;
}

static scpi_result_t cmd_ctrl_sp_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultFloat(context, ctrl_get_setpoint_value((uint8_t)channel));
    return SCPI_RES_OK;
}

scpi_result_t cmd_ctrl_sp(scpi_t *context)
{
    float setpoint;
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamFloat(context, &setpoint, TRUE))
    {
        return SCPI_RES_ERR;
    }

    ctrl_set_setpoint_value((uint8_t)channel, setpoint);
    adq_start((uint8_t)channel, ADQ_TRIGGER_SETPOIT);
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_kp_q(scpi_t *context)
{
    int32_t channel;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    SCPI_ResultFloat(context, ctrl_get_kp_value((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_kp(scpi_t *context)
{
    float kp;
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (SCPI_ParamFloat(context, &kp, true))
    {
        ctrl_set_kp_value((uint8_t)channel, kp);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}

static scpi_result_t cmd_pid_ki_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }
    SCPI_ResultFloat(context, ctrl_get_ki_value((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_ki(scpi_t *context)
{
    int32_t channel;
    float ki;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (SCPI_ParamFloat(context, &ki, true))
    {
        ctrl_set_ki_value((uint8_t)channel, ki);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}

static scpi_result_t cmd_pid_kd_q(scpi_t *context)
{
    int32_t channel;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultFloat(context, ctrl_get_kd_value((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_kd(scpi_t *context)
{
    int32_t channel;
    float kd;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (SCPI_ParamFloat(context, &kd, true))
    {
        ctrl_set_kd_value((uint8_t)channel, kd);
        return SCPI_RES_OK;
    }
    return SCPI_RES_ERR;
}

static scpi_result_t cmd_pid_duty_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultFloat(context, ctrl_get_duty_value((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_type_q(scpi_t *context)
{
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (ctrl_get_controller_type((uint8_t)channel) == CTRL_OPEN)
    {
        SCPI_ResultCharacters(context, "OPEN", 4);
    }
    if (ctrl_get_controller_type((uint8_t)channel) == CTRL_PID)
    {
        SCPI_ResultCharacters(context, "PID", 3);
    }

    return SCPI_RES_OK;
}

static scpi_result_t cmd_pid_type(scpi_t *context)
{
    const char *param_ptr;
    size_t param_len;

    int32_t channel;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamCharacters(context, &param_ptr, &param_len, TRUE))
    {
        return SCPI_RES_ERR;
    }

    if (strncmp(param_ptr, "OPEN", param_len) == 0 && param_len == 4)
    {
        ctrl_set_controller_type((uint8_t)channel, CTRL_OPEN);
    }
    else if (strncmp(param_ptr, "PID", param_len) == 0 && param_len == 3)
    {
        ctrl_set_controller_type((uint8_t)channel, CTRL_PID);
    }

    return SCPI_RES_OK;
}

static scpi_result_t cmd_sour_outp_q(scpi_t *context)
{
    int32_t channel;
    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    SCPI_ResultBool(context, ctrl_get_output_state((uint8_t)channel));
    return SCPI_RES_OK;
}

static scpi_result_t cmd_sour_outp(scpi_t *context)
{
    scpi_bool_t on;
    int32_t channel;

    if (get_root_channel(context, &channel) != SCPI_RES_OK)
    {
        SCPI_ErrorPush(context, SCPI_ERROR_INVALID_SUFFIX);
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamBool(context, &on, true))
    {
        return SCPI_RES_ERR;
    }
    ctrl_output_state((uint8_t)channel, (bool)on);
    ctrl_set_output_state((uint8_t)channel, (bool)on);

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
    {.pattern = "*CLS", .callback = custom_SCPI_CoreCls, .tag = 0},
    {.pattern = "*ESE", .callback = SCPI_CoreEse, .tag = 0},
    {.pattern = "*ESE?", .callback = SCPI_CoreEseQ, .tag = 0},
    {.pattern = "*ESR?", .callback = SCPI_CoreEsrQ, .tag = 0},
    {.pattern = "*IDN?", .callback = SCPI_CoreIdnQ, .tag = 0},
    {.pattern = "*OPC", .callback = SCPI_CoreOpc, .tag = 0},
    {.pattern = "*OPC?", .callback = SCPI_CoreOpcQ, .tag = 0},
    {.pattern = "*RST", .callback = custom_CoreRst, .tag = 0}, //        .callback = SCPI_CoreRst,
    {.pattern = "*SRE", .callback = SCPI_CoreSre, .tag = 0},
    {.pattern = "*SRE?", .callback = SCPI_CoreSreQ, .tag = 0},
    {.pattern = "*STB?", .callback = SCPI_CoreStbQ, .tag = 0},
    {.pattern = "*TST?", .callback = My_CoreTstQ, .tag = 0},
    {.pattern = "*WAI", .callback = SCPI_CoreWai, .tag = 0},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ, .tag = 0},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ, .tag = 0},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ, .tag = 0},
    {.pattern = "STATus:OPERation?", .callback = SCPI_StatusOperationEventQ, .tag = 0},
    {.pattern = "STATus:OPERation:EVENt?", .callback = SCPI_StatusOperationEventQ, .tag = 0},
    {.pattern = "STATus:OPERation:CONDition?", .callback = SCPI_StatusOperationConditionQ, .tag = 0},
    {.pattern = "STATus:OPERation:ENABle", .callback = SCPI_StatusOperationEnable, .tag = 0},
    {.pattern = "STATus:OPERation:ENABle?", .callback = SCPI_StatusOperationEnableQ, .tag = 0},
    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ, .tag = 0},
    /* {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,}, */
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable, .tag = 0},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ, .tag = 0},

    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset, .tag = 0},
    /* Medición */
    {.pattern = "MEASure:SPEED#?", .callback = cmd_meas_speed_q, .tag = 0},
    {.pattern = "MEASure:DATA#?", .callback = cmd_meas_data_q, .tag = 0},
    /* Adquisición*/
    {.pattern = "SOURce#:INITialize", .callback = cmd_adq_init, .tag = 0},
    {.pattern = "SOURce#:ABORT", .callback = cmd_adq_abort, .tag = 0},
    {.pattern = "SOURce#:FETCh?", .callback = cmd_adq_fetch_q, .tag = 0},
    {.pattern = "SOURce#:COUNT?", .callback = cmd_adq_count_q, .tag = 0},
    {.pattern = "SOURce#:TRIGger:SOURce?", .callback = cmd_adq_trig_cfg_q, .tag = 0},
    {.pattern = "SOURce#:TRIGger:SOURce", .callback = cmd_adq_trig_cfg, .tag = 0},
    {.pattern = "SOURce#:DATA:CLEAR", .callback = cmd_adq_data_cls, .tag = 0},
    {.pattern = "SOURce#:DATA:PRETRIGger?", .callback = cmd_adq_data_pre_q, .tag = 0},
    {.pattern = "SOURce#:DATA:PRETRIGger", .callback = cmd_adq_data_pre, .tag = 0},
    {.pattern = "SOURce#:DATA:POINTS", .callback = cmd_adq_data_pts, .tag = 0},
    {.pattern = "SOURce#:DATA:POINTS?", .callback = cmd_adq_data_pts_q, .tag = 0},
    {.pattern = "SOURce#:DATA:FORMat", .callback = cmd_adq_data_format, .tag = 0},
    {.pattern = "SOURce#:DATA:FORMat?", .callback = cmd_adq_data_format_q, .tag = 0},
    /* Setpoint */
    {.pattern = "SOURce#:CONTrol:SETPOint?", .callback = cmd_ctrl_sp_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:SETPOint", .callback = cmd_ctrl_sp, .tag = 0},
    /* PID */
    {.pattern = "SOURce#:CONTrol:PID:KP?", .callback = cmd_pid_kp_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KP", .callback = cmd_pid_kp, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KI?", .callback = cmd_pid_ki_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KI", .callback = cmd_pid_ki, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KD?", .callback = cmd_pid_kd_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:KD", .callback = cmd_pid_kd, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:DUty?", .callback = cmd_pid_duty_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:TYPE?", .callback = cmd_pid_type_q, .tag = 0},
    {.pattern = "SOURce#:CONTrol:PID:TYPE", .callback = cmd_pid_type, .tag = 0},

    /* Salidas */
    {.pattern = "SOURce#:OUTPut", .callback = cmd_sour_outp, .tag = 0},
    {.pattern = "SOURce#:OUTPut?", .callback = cmd_sour_outp_q, .tag = 0},

    SCPI_CMD_LIST_END};

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
                SCPI_RegSetBits(&scpi_context, SCPI_REG_STB, STB_MAV); // ya hay daatos para ser enviados
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
            SCPI_RegClearBits(&scpi_context, SCPI_REG_STB, STB_MAV); // ¡MAV se apaga al terminar!
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