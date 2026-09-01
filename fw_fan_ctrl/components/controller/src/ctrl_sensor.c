#include "driver/mcpwm_cap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "ctrl_data.h"
#include "ctrl_runtime.h"
#include "ctrl_sensor.h"

#define RPM_CONST_COEFICIENT (60.0f * (CTRL_SENSOR_TIMER_RES_HZ) / (CTRL_SENSOR_PPR))
#define SENSOR_TIMER_TICKS_TO_US CTRL_SENSOR_TIMER_RES_HZ / 1000000UL
#define CTRL_TASK_STACK_SIZE 4 * 1024
#define TASK_SPEED_EVENT_TIMEOUT_MS 80
#define MIN_SPEED_RPM_CUTOFF 6.0f

const static char *TAG = "ctrl_capture";

static capture_edges_t edge_history[CTRL_NUM_CHANNELS];
static isr_to_task_data_t shared_data[CTRL_NUM_CHANNELS];
// Estructura estatica de la interrupción de captura
static mcpwm_cap_timer_handle_t cap_timer_handle = NULL;
static portMUX_TYPE sensor_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Estructura de la tarea de actualización de la velocidad
static StaticTask_t x_ctrl_sensor_event_TaskBuffer;
static StackType_t x_ctrl_sensor_event_TaskStack[CTRL_TASK_STACK_SIZE];
static TaskHandle_t x_ctrl_sensor_event_TaskHandle = NULL;

static float filtred_speed[CTRL_NUM_CHANNELS] = {0.0f};
static float current_speed[CTRL_NUM_CHANNELS] = {0.0f};

static bool isr_captura(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    uint8_t canal = (uint8_t)(uintptr_t)user_data;
    uint32_t last_event = edata->cap_value;
    uint32_t period = 0;
    BaseType_t xHigherPriorityTaskMayWake = pdFALSE;

    portENTER_CRITICAL_ISR(&sensor_spinlock);

    period = last_event - edge_history[canal].last_edges[1];

    edge_history[canal].last_edges[1] = edge_history[canal].last_edges[0];
    edge_history[canal].last_edges[0] = last_event;

    shared_data[canal].period_ticks = period;

    shared_data[canal].timestamp_hw_us = esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&sensor_spinlock);

    if (x_ctrl_sensor_event_TaskHandle != NULL)
    {
        xTaskNotifyFromISR(x_ctrl_sensor_event_TaskHandle, (1 << canal), eSetBits, &xHigherPriorityTaskMayWake);
    }

    return xHigherPriorityTaskMayWake == pdTRUE;
}

static bool ctrl_make_sample(float speed, uint64_t timestamp_us, ctrl_sensor_data_t *out)
{
    if (out)
    {
        out->speed = speed;
        out->timestamp_us = timestamp_us;
        return true;
    }
    return false;
}

/* Filtrado de datos de velocidad (usado por la pantalla) */
#define FILTER_ORDER 5
static float alpha, beta;
static float ema_speed[CTRL_NUM_CHANNELS][FILTER_ORDER];

static float get_alpha(void)
{
    double fc = 4;
    double fm = 40;
    double correccion_cascada = sqrt(pow(2, 1.0 / (double)FILTER_ORDER) - 1.0);
    double alpha = 1.0 - exp(-2.0 * M_PI * (fc / fm) / correccion_cascada);
    return (float)alpha;
}

static inline void update_filtred_speed(uint8_t channel, float speed)
{
    float local_ema[FILTER_ORDER];

    portENTER_CRITICAL(&sensor_spinlock);
    for (uint8_t i = 0; i < FILTER_ORDER; i++)
    {
        local_ema[i] = ema_speed[channel][i];
    }
    portEXIT_CRITICAL(&sensor_spinlock);

    local_ema[0] = alpha * local_ema[0] + beta * speed;
    for (uint8_t i = 1; i < FILTER_ORDER; i++)
    {
        local_ema[i] = alpha * local_ema[i] + beta * local_ema[i - 1];
    }

    portENTER_CRITICAL(&sensor_spinlock);
    for (uint8_t i = 0; i < FILTER_ORDER; i++)
    {
        ema_speed[channel][i] = local_ema[i];
    }
    filtred_speed[channel] = local_ema[FILTER_ORDER - 1];
    portEXIT_CRITICAL(&sensor_spinlock);
}

// Esta funcion se llama desde la tarea de control periodica cuando es despertada por un evento de captura. Para no calcular la velocidad dentro de la ISR se calculara y guardara acá.
static void ctrl_asinc_update_speed(uint8_t channel)
{
    uint32_t period = 0;
    uint64_t last_pulse_us = 0;
    ctrl_buffer_data_t *buffer = ctrl_get_buffers(channel);
    ctrl_sensor_data_t item;

    portENTER_CRITICAL(&sensor_spinlock);
    period = shared_data[channel].period_ticks;
    last_pulse_us = shared_data[channel].timestamp_hw_us;
    current_speed[channel] = RPM_CONST_COEFICIENT / (float)period;
    portEXIT_CRITICAL(&sensor_spinlock);

    ctrl_make_sample(current_speed[channel], last_pulse_us, &item);
    update_filtred_speed(channel, current_speed[channel]);

    // Se inyecta en el buffer el nuevo dato de la velocidad con su marca de tiempo
    buffer->push(buffer, &item);
}

static float ctrl_get_bounded_speed(uint8_t channel)
{
    uint32_t period = 0;
    uint64_t last_pulse_us = 0;

    portENTER_CRITICAL(&sensor_spinlock);
    period = shared_data[channel].period_ticks;
    last_pulse_us = shared_data[channel].timestamp_hw_us;
    portEXIT_CRITICAL(&sensor_spinlock);

    uint64_t time = esp_timer_get_time();

    // si todavia no pasaron 50ms usamos la velocidad media, si pasaron mas de 50ms recalculamos la cota superior de la velocidad
    float bounded_speed;

    if (time < (last_pulse_us + TASK_SPEED_EVENT_TIMEOUT_MS * 1000))
    {
        bounded_speed = ctrl_get_filtred_speed(channel);
    }
    else
    {
        uint64_t bounded_period = SENSOR_TIMER_TICKS_TO_US * (time - last_pulse_us) + period;
        bounded_speed = RPM_CONST_COEFICIENT / (float)bounded_period;
    }

    // si la cota de velocidad  es menor a 6RPM se pone la velocidad a 0
    if (bounded_speed < MIN_SPEED_RPM_CUTOFF)
    {
        bounded_speed = 0.0f;
    }

    update_filtred_speed(channel, bounded_speed);
    return bounded_speed;
}

float ctrl_get_sensor_speed(uint8_t channel)
{
    if (channel < CTRL_NUM_CHANNELS)
    {
        return current_speed[channel];
    }
    return 0.0f;
}

float ctrl_get_filtred_speed(uint8_t channel)
{
    return filtred_speed[channel];
}

static inline bool is_channel_in_timeout(uint8_t channel, uint64_t actual_time_us)
{
    uint64_t last_pulse_us;

    portENTER_CRITICAL(&sensor_spinlock);
    last_pulse_us = shared_data[channel].timestamp_hw_us;
    portEXIT_CRITICAL(&sensor_spinlock);

    return ((actual_time_us - last_pulse_us) > (TASK_SPEED_EVENT_TIMEOUT_MS * 1000ULL));
}

static inline uint32_t get_last_period(uint8_t channel)
{
    uint32_t ticks0, ticks1;
    portENTER_CRITICAL(&sensor_spinlock);
    ticks0 = edge_history[channel].last_edges[0];
    ticks1 = edge_history[channel].last_edges[1];
    portEXIT_CRITICAL(&sensor_spinlock);
    return ticks0 - ticks1;
}

void ctrl_sensor_event_task(void *vpParameter)
{
    uint32_t eventos_notificados = 0;

    while (1)
    {
        // El timeout se usa para revisar canales inactivos
        BaseType_t xStatus = xTaskNotifyWait(0x00, 0xFFFFFFFF, &eventos_notificados, pdMS_TO_TICKS(TASK_SPEED_EVENT_TIMEOUT_MS));
        uint64_t actual_time_us = esp_timer_get_time();

        for (uint8_t channel = 0; channel < CTRL_NUM_CHANNELS; channel++)
        {
            // 1. Revisar si ESTE canal generó el evento que despertó la tarea
            if ((xStatus == pdTRUE) && (eventos_notificados & (1UL << channel)))
            {
                // Actualiza buffers y filtro
                ctrl_asinc_update_speed(channel);

                float last_time_interval = (float)(get_last_period(channel)) / (float)CTRL_SENSOR_TIMER_RES_HZ;

                // TODO: llamar a la rutina de control para el canal con la velocidad real y el periodo
                ctrl_fsm_runtime(channel, current_speed[channel], last_time_interval);
            }
            // 2. Si NO generó evento, se verifica el timeout del canal
            else if (is_channel_in_timeout(channel, actual_time_us))
            {
                float velocidad_acotada = ctrl_get_bounded_speed(channel);
                uint64_t last_pulse_us;

                portENTER_CRITICAL(&sensor_spinlock);
                last_pulse_us = shared_data[channel].timestamp_hw_us;
                portEXIT_CRITICAL(&sensor_spinlock);

                float last_time_interval = (float)(get_last_period(channel)) / (float)CTRL_SENSOR_TIMER_RES_HZ + (actual_time_us - last_pulse_us) / 1000000.0f;

                // TODO: llamar a la rutina de control para el canal con la velocidad acotada
                ctrl_fsm_runtime(channel, velocidad_acotada, last_time_interval);
            }
        }
    }
}

void ctrl_sensor_init(void)
{
    alpha = get_alpha();
    beta = 1.0f - alpha;
    ESP_LOGW(TAG, "Periferico inicializado");
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0};

    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer_handle));

    for (int i = 0; i < CTRL_NUM_CHANNELS; i++)
    {
        mcpwm_capture_channel_config_t chan_conf = {
            .gpio_num = 1 + i,
            .prescale = 1,
            .flags.neg_edge = true,
            .flags.pos_edge = true};

        mcpwm_cap_channel_handle_t cap_chan;
        ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer_handle, &chan_conf, &cap_chan));

        mcpwm_capture_event_callbacks_t cbs_chan = {
            .on_cap = isr_captura,
        };

        ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs_chan, (void *)(uintptr_t)i));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
    }
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer_handle));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer_handle));

    x_ctrl_sensor_event_TaskHandle = xTaskCreateStaticPinnedToCore(
        ctrl_sensor_event_task,          // Puntero a la función de la tarea
        "Controller Task",               // Nombre (para depuración)
        CTRL_TASK_STACK_SIZE,            // Tamaño de la pila
        NULL,                            // Parámetro de entrada
        10,                              // Prioridad
        x_ctrl_sensor_event_TaskStack,   // Arreglo estático para la pila
        &x_ctrl_sensor_event_TaskBuffer, // Estructura estática para el TCB
        0                                // Anclado al Core 1 (APP_CPU)
    );
}
