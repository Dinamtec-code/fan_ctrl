#include "ctrl_sensor.h"
#include "driver/mcpwm_cap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ctrl_data.h"
#include "stdint.h"
#include "stdbool.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

extern TaskHandle_t x_ctrl_TaskHandle;

const static char *TAG = "ctrl_capture";

static capture_edges_t edge_history[CTRL_NUM_CHANNELS];
static isr_to_task_data_t shared_data[CTRL_NUM_CHANNELS];
static mcpwm_cap_timer_handle_t cap_timer_handle = NULL;
static portMUX_TYPE sensor_spinlock = portMUX_INITIALIZER_UNLOCKED;

static bool isr_captura(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    uint8_t canal = (uint8_t)(uintptr_t)user_data;
    uint32_t last_event = edata->cap_value;
    uint32_t period = 0;
    BaseType_t xHigherPriorityTaskMayWake = pdFALSE;

    taskENTER_CRITICAL_ISR(&sensor_spinlock);

    period = last_event - edge_history[canal].last_edges[1];

    edge_history[canal].last_edges[1] = edge_history[canal].last_edges[0];
    edge_history[canal].last_edges[0] = last_event;

    shared_data[canal].period_ticks = period;

    // Novedad: Guardamos el tiempo del sistema (64 bits reales, nunca desborda en la vida útil)
    shared_data[canal].timestamp_hw_us = esp_timer_get_time();

    taskEXIT_CRITICAL_ISR(&sensor_spinlock);

    if (x_ctrl_TaskHandle != NULL)
    {
        xTaskNotifyFromISR(x_ctrl_TaskHandle, (1 << canal), eSetBits, &xHigherPriorityTaskMayWake);
    }

    return xHigherPriorityTaskMayWake == pdTRUE;
}

static inline void send_speed(float speed, uint8_t channel)
{
    taskENTER_CRITICAL(&sensor_spinlock);
    shared_data[channel].speed = speed;
    taskEXIT_CRITICAL(&sensor_spinlock);
}

float ctrl_get_sensor_speed(uint8_t channel)
{

    if (channel >= CTRL_NUM_CHANNELS)
    {
        return 0.0f;
    }
    uint32_t period = 0;
    uint64_t ultimo_pulso_us = 0;

    taskENTER_CRITICAL(&sensor_spinlock);
    period = shared_data[channel].period_ticks;
    ultimo_pulso_us = shared_data[channel].timestamp_hw_us;
    taskEXIT_CRITICAL(&sensor_spinlock);

    if (period == 0)
    {
        return 0.0f;
    }
    // Manejo de timeout usando el tiempo absoluto
    uint64_t tiempo_actual_us = esp_timer_get_time();

    // Ajusta CTRL_SENSOR_TIMEOUT_TICKS a su equivalente en microsegundos si es necesario
    if ((tiempo_actual_us - ultimo_pulso_us) > CTRL_SENSOR_TIMEOUT_TICKS)
    {
        return 0.0f;
    }

    taskENTER_CRITICAL(&sensor_spinlock);
    period = shared_data[channel].period_ticks;
    ultimo_pulso_us = shared_data[channel].timestamp_hw_us;
    taskEXIT_CRITICAL(&sensor_spinlock);

    float speed = (60.0f * (float)CTRL_SENSOR_TIMER_RES_HZ) / ((float)period * (float)CTRL_SENSOR_PPR);
    ctrl_buffer_data_t *buffer = &ctrl_sensor_buffers[channel];
    buffer->push(buffer, speed, shared_data[channel].timestamp_hw_us);
    ESP_LOGW(TAG, "velocidad medida: %f", speed);
    return speed;
}

void ctrl_sensor_init(void)
{
    ESP_LOGW(TAG, "Periferico inicializado");
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0};

    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer_handle));

    for (int i = 0; i < CTRL_NUM_CHANNELS; i++)
    {
        mcpwm_capture_channel_config_t chan_conf = {
            .gpio_num = 40 + i,
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
}