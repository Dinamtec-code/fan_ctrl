#ifndef ADQ_RUNTIME_H_
#define ADQ_RUNTIME_H_
#include "ctrl_sensor.h" //aca defini el tipo de dato que almacena el buffer de adquicisión
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Configuración de la tarea de adquisición
 **/
#define ADQ_TASK_STACK_SIZE 4096
#define ADQ_TASK_PRIORITY 5
#define ADQ_TASK_CORE 0
#define ADQ_TASK_NAME "adq_task"

/**
 * Configuración del buffer de adquisición.
 * TODO: para incrementar el buffer luego se puede usar la PSRAM del microcontrolados)
 **/
#define ADQ_BUFFER_SIZE 1024 * 64

/**
 * Banderas de la notificación de la tarea de adquisición
 */
#define ADQ_NOTIFY_INIT BIT0
#define ADQ_NOTIFY_TRIGGER BIT1
#define ADQ_NOTIFY_ABORT BIT2
#define ADQ_NOTIFY_PROCESS BIT3
#define ADQ_NOTIFY_UPDATE_CFG BIT4
#define ADQ_NOTIFY_COMPLETE BIT5

/**
 * Banderas del canal que genero la notrifoicación
 */
#define ADQ_NOTIFY_CH0 BIT15
#define ADQ_NOTIFY_CH1 BIT16

    typedef enum
    {
        ADQ_STATE_STOPPED,
        ADQ_STATE_WAITING,
        ADQ_STATE_RUNNING,
        ADQ_STATE_ERROR,
        ADQ_STATE_COUNT
    } adq_state_t;

    typedef enum
    {
        ADQ_TRIGGER_NONE,
        ADQ_TRIGGER_SETPOIT,
        ADQ_TRIGGER_OUTPUT_ON,
        ADQ_TRIGGER_EXTERNAL,
        ADQ_TRIGGER_SOFTWARE,
        ADQ_TRIGGER_COUNT
    } adq_trigger_source_t;

    typedef enum
    {
        ADQ_COMPLETE_NONE,
        ADQ_COMPLETE_POINTS,
        ADQ_COMPLETE_TIME,
        ADQ_COMPLETE_ABORT,
        ADQ_COMPLETE_COUT
    } adq_complete_event_t;

    // Function declarations for runtime control functions
    void adq_system_init(void);
    bool adq_cfg_update(uint8_t channel);
    bool adq_adq_initialize(uint8_t channel);
    bool adq_start(uint8_t channel, adq_trigger_source_t source);
    bool adq_new_data_available(uint8_t channel);
    bool adq_stop(uint8_t channel);
    size_t adq_get_raw_buffer(ctrl_sensor_data_t *target_buffer, uint8_t channel, size_t max_size);
    size_t adq_get_count(uint8_t channel);
    bool adq_data_clear(uint8_t channel);
    bool adq_is_running(uint8_t channel);
    bool adq_is_waiting(uint8_t channel);
    bool adq_is_stopped(uint8_t channel);
    bool adq_is_error(uint8_t channel);
    adq_state_t adq_get_state(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif // ADQ_RUNTIME_H_