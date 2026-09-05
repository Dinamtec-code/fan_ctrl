#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include "adq_cfg.h"
#include "ctrl_data.h"

static const char *TAG = "adq_runtime";

/**
 * Memoria y estructura estatica para la tarea de adquisisión
 */
static StaticTask_t x_adq_taskBuffer;
static StackType_t x_adq_taskStack[ADQ_TASK_STACK_SIZE];
static TaskHandle_t x_adq_taskHandle = NULL;

typedef enum
{
    ADQ_EVENT_NONE = 0,
    ADQ_EVENT_INIT = ADQ_NOTIFY_INIT,
    ADQ_EVENT_TRIGGER = ADQ_NOTIFY_TRIGGER,
    ADQ_EVENT_ABORT = ADQ_NOTIFY_ABORT,
    ADQ_EVENT_PROCESS = ADQ_NOTIFY_PROCESS,
    ADQ_EVENT_UPDATE_CFG = ADQ_NOTIFY_UPDATE_CFG,
    ADQ_EVENT_COMPLETE = ADQ_NOTIFY_COMPLETE
} adq_event_t;

typedef enum
{
    ADQ_EVENT_CHANNEL_NONE = 0,
    ADQ_EVENT_CHANNEL_0 = ADQ_NOTIFY_CH0,
    ADQ_EVENT_CHANNEL_1 = ADQ_NOTIFY_CH1
} adq_event_channel_t;

// estructura del búfer lineal para guardado de los datos
typedef struct ADQ_BUFFER
{
    ctrl_sensor_data_t *buffer;
    size_t buffer_size;
    size_t count;
    uint64_t start_time_us;
} adq_buffer_t;

typedef struct ADQ_CFG
{
    uint32_t pretrigger;
    uint32_t points;
    adq_complete_event_t complete_condition;
    adq_trigger_source_t trigger_source;
} adq_cfg_t;

static adq_state_t adq_state[CTRL_NUM_CHANNELS] = {ADQ_STATE_STOPPED};

static ctrl_sensor_data_t *adq_buffers_mem[CTRL_NUM_CHANNELS];

static adq_buffer_t adq_buffer[CTRL_NUM_CHANNELS] = {
    [0] = {// Limpieza del primer buffer
           .buffer = NULL,
           .buffer_size = 0,
           .count = 0,
           .start_time_us = 0},
    [1] = {// Limpieza del segundo buffer
           .buffer = NULL,
           .buffer_size = 0,
           .count = 0,
           .start_time_us = 0}};

static adq_cfg_t adq_cfg[CTRL_NUM_CHANNELS] = {
    [0] = {.pretrigger = 10,
           .points = 100,
           .complete_condition = ADQ_COMPLETE_POINTS,
           .trigger_source = ADQ_TRIGGER_SETPOIT},
    [1] = {.pretrigger = 10,
           .points = 100,
           .complete_condition = ADQ_COMPLETE_POINTS,
           .trigger_source = ADQ_TRIGGER_SETPOIT}};

uint32_t xHigherPriorityTaskMayWake = pdFALSE;
uint64_t adq_start_time_us = 0;

static void get_event_from_notify(uint32_t *notify_value, adq_event_t *event, adq_event_channel_t *channels)
{
    *event = ADQ_EVENT_NONE;
    *channels = ADQ_EVENT_CHANNEL_NONE;
    if (*notify_value & ADQ_NOTIFY_INIT)
    {
        *event = ADQ_EVENT_INIT;
        *notify_value &= ~ADQ_NOTIFY_INIT;
    }
    if (*notify_value & ADQ_NOTIFY_TRIGGER)
    {
        *event = ADQ_EVENT_TRIGGER;
        *notify_value &= ~ADQ_NOTIFY_TRIGGER;
    }
    if (*notify_value & ADQ_NOTIFY_ABORT)
    {
        *event = ADQ_EVENT_ABORT;
        *notify_value &= ~ADQ_NOTIFY_ABORT;
    }
    if (*notify_value & ADQ_NOTIFY_PROCESS)
    {
        *event = ADQ_EVENT_PROCESS;
        *notify_value &= ~ADQ_NOTIFY_PROCESS;
    }
    if (*notify_value & ADQ_NOTIFY_UPDATE_CFG)
    {
        *event = ADQ_EVENT_UPDATE_CFG;
        *notify_value &= ~ADQ_NOTIFY_UPDATE_CFG;
    }
    if (*notify_value & ADQ_NOTIFY_CH0)
    {
        *channels |= ADQ_EVENT_CHANNEL_0;
        *notify_value &= ~ADQ_NOTIFY_CH0;
    }
    if (*notify_value & ADQ_NOTIFY_CH1)
    {
        *channels |= ADQ_EVENT_CHANNEL_1;
        *notify_value &= ~ADQ_NOTIFY_CH1;
    }
}

static bool adq_is_capture_complete(uint8_t channel)
{
    bool complete = false;
    if (adq_cfg[channel].complete_condition == ADQ_COMPLETE_POINTS)
    {
        if (adq_buffer[channel].count >= adq_cfg[channel].points)
        {
            complete = true;
        }
    }
    else if (adq_cfg[channel].complete_condition == ADQ_COMPLETE_TIME)
    {
        uint64_t current_time_us = esp_timer_get_time();
        if ((current_time_us - adq_start_time_us) >= adq_cfg[channel].points)
        {
            complete = true;
        }
    }
    return complete;
}

static inline adq_state_t adq_copy_pretrigger(ctrl_buffer_data_t *source_buffer, uint8_t channel)
{
    adq_state_t state = ADQ_STATE_RUNNING;
    if (adq_buffer[channel].buffer != NULL && adq_buffer[channel].buffer_size > 0)
    {
        ESP_LOGE(TAG, "Se pidio un pre-trigger de: %d. EL buffer tiene: %d", adq_cfg[channel].pretrigger, ctrl_debug_data_buffer_count(source_buffer));
        if (!source_buffer->seek_tail(source_buffer, adq_cfg[channel].pretrigger))
        {
            return ADQ_STATE_ERROR;
        }
        size_t items_to_copy = (adq_cfg[channel].pretrigger < adq_cfg[channel].points) ? adq_cfg[channel].pretrigger : adq_cfg[channel].points;
        source_buffer->pop_block(source_buffer, adq_buffer[channel].buffer, items_to_copy);
        adq_buffer[channel].count = items_to_copy;
    }
    else
    {
        // Handle memory allocation failure
        state = ADQ_STATE_ERROR;
    }

    ESP_LOGE(TAG, "Se copió el pretrigger al buffer. Copiados %4d de %4d", adq_buffer[channel].count, adq_cfg[channel].points);

    return state;
}

inline static void debug_event(adq_event_t event, uint8_t channel)
{
    switch (event)
    {
    case ADQ_EVENT_NONE:
        break;
    case ADQ_EVENT_INIT:
        ESP_LOGE(TAG, "Evento  ADQ_EVENT_INIT");
        ESP_LOGI(TAG, "CONFIG: points %d", adq_cfg[channel].points);
        ESP_LOGI(TAG, "CONFIG: pretri %d", adq_cfg[channel].pretrigger);
        ESP_LOGI(TAG, "CONFIG: comple %d", adq_cfg[channel].complete_condition);

        break;
    case ADQ_EVENT_TRIGGER:
        ESP_LOGE(TAG, "Evento: ADQ_EVENT_TRIGGER");
        break;
    case ADQ_EVENT_ABORT:
        ESP_LOGE(TAG, "Evento: ADQ_EVENT_ABORT");
        break;
    case ADQ_EVENT_PROCESS:
        ESP_LOGE(TAG, "Evento: ADQ_EVENT_PROCESS");
        break;
    case ADQ_EVENT_UPDATE_CFG:
        ESP_LOGE(TAG, "Evento: ADQ_EVENT_UPDATE_CFG");
        break;
    case ADQ_EVENT_COMPLETE:
        ESP_LOGE(TAG, "Evento: ADQ_EVENT_COMPLETE");
        break;
    default:
        ESP_LOGE(TAG, "Evento: Desconocido");
        break;
    }
}
static inline void update_cfg(uint8_t channel)
{
    ESP_LOGI(TAG,
             "CH%u CFG: pre=%lu points=%lu complete=%d trigger=%d",
             channel,
             (unsigned long)adq_cfg[channel].pretrigger,
             (unsigned long)adq_cfg[channel].points,
             (int)adq_cfg[channel].complete_condition,
             (int)adq_cfg[channel].trigger_source);
    adq_cfg[channel].pretrigger = ctrl_get_adq_pretrigger_value(channel);
    adq_cfg[channel].points = ctrl_get_adq_points_value(channel);
    adq_cfg[channel].complete_condition = ctrl_get_adq_compete_condition_value(channel);
    adq_cfg[channel].trigger_source = ctrl_get_adq_trigger_source_value(channel);
    ESP_LOGI(TAG,
             "CH%u CFG: pre=%lu points=%lu complete=%d trigger=%d",
             channel,
             (unsigned long)adq_cfg[channel].pretrigger,
             (unsigned long)adq_cfg[channel].points,
             (int)adq_cfg[channel].complete_condition,
             (int)adq_cfg[channel].trigger_source);
}

static void adq_fsm(adq_event_t event, uint8_t channel)
{
    static bool update_pending[CTRL_NUM_CHANNELS] = {0};
    debug_event(event, channel);
    if (adq_state[channel] >= ADQ_STATE_COUNT || channel >= CTRL_NUM_CHANNELS)
    {
        return;
    }

    switch (adq_state[channel])
    {
    case ADQ_STATE_STOPPED:
        if (event == ADQ_EVENT_INIT)
        {
            adq_state[channel] = ADQ_STATE_WAITING;
        }
        else if (event == ADQ_EVENT_NONE)
        {
            if (update_pending[channel] == true)
            {
                ESP_LOGE(TAG, "ejecutando actualización pendiente");
                update_pending[channel] = false;
            }
        }
        else if (event == ADQ_EVENT_UPDATE_CFG)
        {
            ESP_LOGE(TAG, "actualización desde stopped");
            update_cfg(channel);
        }
        break;
    case ADQ_STATE_WAITING:
        if (event == ADQ_EVENT_TRIGGER)
        {
            adq_state[channel] = ADQ_STATE_RUNNING;
            ESP_LOGE(TAG, "Trigger recibido para el canal: %d", channel);

            adq_buffer[channel].start_time_us = esp_timer_get_time();
            ctrl_buffer_data_t *source_buffer = ctrl_get_buffers(channel);
            if (source_buffer->get_count(source_buffer) < adq_cfg[channel].pretrigger)
            {
                // Not enough data for pretrigger, handle error
                ESP_LOGE(TAG, "Not enough data for pretrigger on channel %d", channel);
                // Publicar el error en el parser scpi
                adq_state[channel] = ADQ_STATE_ERROR;
                break;
            }
            adq_state[channel] = adq_copy_pretrigger(source_buffer, channel);

            if (adq_is_capture_complete(channel))
            {
                adq_fsm(ADQ_EVENT_COMPLETE, channel);
            }
        }
        else if (event == ADQ_EVENT_ABORT)
        {
            adq_state[channel] = ADQ_STATE_STOPPED;
        }
        else if (event == ADQ_EVENT_UPDATE_CFG)
        {
            ESP_LOGE(TAG, "actualización desde waiting");
            update_cfg(channel);
        }
        break;
    case ADQ_STATE_RUNNING:
        if (event == ADQ_EVENT_COMPLETE)
        {
            adq_state[channel] = ADQ_STATE_STOPPED;

            /**
             * TODO: Implementar la lógica de finalización de la adquisición
             * informar a la pantalla el cambio de estado del canal
             **/
            // Informar al parser SCPI configurar los estados del equipo y enviar la notificación de finalización de la adquisición
        }
        else if (event == ADQ_EVENT_ABORT)
        {
            adq_state[channel] = ADQ_STATE_STOPPED;
            /**
             * TODO: Implementar la lógica de aborto de la adquisición
             * informar a la pantalla el cambio de estado del canal
             **/
            // Informar al parser SCPI configurar los estados del equipo y enviar la notificación de finalización de la adquisición
        }
        else if (event == ADQ_EVENT_PROCESS)
        {
            ESP_LOGE(TAG, "Copiando %d", channel);
            ctrl_buffer_data_t *source_buffer = ctrl_get_buffers(channel);
            uint32_t new_data_count = source_buffer->pop_block(source_buffer, adq_buffer[channel].buffer + adq_buffer[channel].count, adq_cfg[channel].points - adq_buffer[channel].count);
            adq_buffer[channel].count += new_data_count;
            ESP_LOGE(TAG, "Copiando %d datos mas. Total %d de ", new_data_count, adq_buffer[channel].count, adq_cfg[channel].points);
            if (adq_is_capture_complete(channel))
            {
                adq_fsm(ADQ_EVENT_COMPLETE, channel);
            }
        }
        else if (event == ADQ_EVENT_UPDATE_CFG)
        {
            // Handle configuration update while running
            // Publicar error en el buffer scpi para informar la escritura durante la adquicisión.
            ESP_LOGI(TAG, "Configuration updated while running on channel %d", channel);
            update_pending[channel] = true;
        }
        else if (event == ADQ_EVENT_NONE)
        {
            // Durante la adquisición se revisa periodicamente si hay para liberar el buffer del sensor
            ctrl_buffer_data_t *source_buffer = ctrl_get_buffers(channel);
            uint32_t new_data_count = source_buffer->pop_block(source_buffer, adq_buffer[channel].buffer + adq_buffer[channel].count, adq_cfg[channel].points - adq_buffer[channel].count);
            adq_buffer[channel].count += new_data_count;
            if (adq_is_capture_complete(channel))
            {
                adq_fsm(ADQ_EVENT_COMPLETE, channel);
            }
        }
        break;
    case ADQ_STATE_ERROR:
        if (event == ADQ_EVENT_NONE)
        {
            // Se espera el timeout de la tarea de adq para reiniciar
            adq_state[channel] = ADQ_STATE_STOPPED;
        }
        else if (event == ADQ_EVENT_UPDATE_CFG)
        {
            ESP_LOGE(TAG, "actualización desde error");
            update_cfg(channel);
        }
        break;
    default:
        if (event == ADQ_EVENT_UPDATE_CFG)
        {
            ESP_LOGE(TAG, "actualización desde default");
            update_cfg(channel);
        }
        return;
    }
}

static void adq_task(void *pvParameters)
{
    uint32_t comm_notify = 0;
    adq_event_t event = ADQ_EVENT_NONE;
    adq_event_channel_t channels_event = ADQ_EVENT_CHANNEL_NONE;
    while (1)
    {

        BaseType_t xStatus = xTaskNotifyWait(0x00, 0xFFFFFFFF, &comm_notify, pdMS_TO_TICKS(100));
        if (xStatus == pdTRUE)
        {
            get_event_from_notify(&comm_notify, &event, &channels_event);
            if (channels_event == ADQ_EVENT_CHANNEL_NONE)
            {
                // Handle the case where no channel is specified
                continue;
            }
            if (channels_event & ADQ_EVENT_CHANNEL_0)
            {
                adq_fsm(event, 0);
            }
            if (channels_event & ADQ_EVENT_CHANNEL_1)
            {
                adq_fsm(event, 1);
            }
        }
        else
        {
            for (int i = 0; i < CTRL_NUM_CHANNELS; i++)
            {
                if (ctrl_adq_data_get_update(i))
                {
                    adq_fsm(ADQ_EVENT_UPDATE_CFG, i);
                }
                adq_fsm(ADQ_EVENT_NONE, i);
            }
        }
    }
}

// Function declarations for runtime control functions
void adq_system_init(void)
{
    for (int ch = 0; ch < CTRL_NUM_CHANNELS; ch++)
    {
        // Asignacion interna del buffer a la PSRAM.
        adq_buffers_mem[ch] = heap_caps_malloc(ADQ_BUFFER_SIZE * sizeof(ctrl_sensor_data_t), MALLOC_CAP_SPIRAM);
        if (adq_buffers_mem[ch] == NULL)
        {
            ESP_LOGW(TAG, "PSRAM allocation failed for channel %d, falling back to DRAM", ch);
            adq_buffers_mem[ch] = malloc(ADQ_BUFFER_SIZE * sizeof(ctrl_sensor_data_t)); // DRAM
            if (adq_buffers_mem[ch] == NULL)
            {
                ESP_LOGE(TAG, "Failed to allocate buffer for channel %d", ch);
                continue; // o manejar error crítico
            }
        }
        adq_buffer[ch].buffer = adq_buffers_mem[ch];
        adq_buffer[ch].buffer_size = ADQ_BUFFER_SIZE;
        adq_buffer[ch].count = 0;
    }

    // Crear la tarea de adquisición (como ya lo haces)
    x_adq_taskHandle = xTaskCreateStatic(adq_task, ADQ_TASK_NAME, ADQ_TASK_STACK_SIZE,
                                         NULL, ADQ_TASK_PRIORITY, x_adq_taskStack, &x_adq_taskBuffer);
}

inline static bool notify_event(uint8_t channel, adq_event_t event)
{
    if (x_adq_taskHandle != NULL)
    {
        if (channel == 0)
        {
            xTaskNotify(x_adq_taskHandle, event | ADQ_NOTIFY_CH0, eSetBits);
        }
        else if (channel == 1)
        {
            xTaskNotify(x_adq_taskHandle, event | ADQ_NOTIFY_CH1, eSetBits);
        }
        else
        {
            // Handle the case where the channel is invalid
            return false;
        }
    }
    else
    {
        // Handle the case where the task handle is NULL
        return false;
    }
    return true;
}

bool adq_cfg_update(uint8_t channel)
{
    return notify_event(channel, ADQ_NOTIFY_UPDATE_CFG);
}

bool adq_adq_initialize(uint8_t channel)
{
    return notify_event(channel, ADQ_NOTIFY_INIT);
}

bool adq_start(uint8_t channel, adq_trigger_source_t source)
{
    if (channel >= CTRL_NUM_CHANNELS)
        return false;

    if (source == adq_cfg[channel].trigger_source)
    {
        return notify_event(channel, ADQ_NOTIFY_TRIGGER);
    }

    return false;
}

bool adq_stop(uint8_t channel)
{
    return notify_event(channel, ADQ_NOTIFY_ABORT);
}

bool adq_new_data_available(uint8_t channel)
{
    return notify_event(channel, ADQ_NOTIFY_PROCESS);
}

size_t adq_get_raw_buffer(ctrl_sensor_data_t *target_buffer, uint8_t channel, size_t max_size)
{
    if (target_buffer == NULL || max_size == 0 || adq_buffer[channel].buffer == NULL)
    {
        return 0;
    }
    if (max_size > adq_buffer[channel].count)
    {
        max_size = adq_buffer[channel].count;
    }
    memcpy(target_buffer, adq_buffer[channel].buffer, max_size * sizeof(ctrl_sensor_data_t));

    return max_size;
}

size_t adq_get_count(uint8_t channel)
{
    if (adq_buffer[channel].buffer == NULL)
    {
        return 0;
    }

    return adq_buffer[channel].count;
}

bool adq_data_clear(uint8_t channel)
{
    if (channel >= CTRL_NUM_CHANNELS)
        return false;

    adq_buffer->count = 0;
    return true;
}

bool adq_is_running(uint8_t channel)
{
    if (adq_state[channel] == ADQ_STATE_RUNNING)
    {
        return true;
    }
    return false;
}

bool adq_is_waiting(uint8_t channel)
{

    if (adq_state[channel] == ADQ_STATE_WAITING)
    {
        return true;
    }
    return false;
}

bool adq_is_stopped(uint8_t channel)
{

    if (adq_state[channel] == ADQ_STATE_STOPPED)
    {
        return true;
    }
    return false;
}

bool adq_is_error(uint8_t channel)
{
    if (adq_state[channel] == ADQ_STATE_ERROR)
    {
        return true;
    }
    return false;
}

adq_state_t adq_get_state(uint8_t channel)
{
    return adq_state[channel];
}