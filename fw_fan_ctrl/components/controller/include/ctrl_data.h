#ifndef CTRL_DATA_H_
#define CTRL_DATA_H_
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"

#include "ctrl_sensor.h"
#include "adq_cfg.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SENSOR_BUFFER_SIZE 1024
#define CTRL_NUM_CHANNELS 2

    typedef enum
    {
        CTRL_OPEN,
        CTRL_PID
    } ctrl_type_t;

    typedef struct
    {
        float value;
        float max;
        float min;
    } ctrl_param_t;

    typedef struct CTRL_BUFFER_DATA ctrl_buffer_data_t;

    struct CTRL_BUFFER_DATA
    {
        ctrl_sensor_data_t *data;
        size_t capacity;
        size_t head;
        size_t tail;
        bool overflow_flag;
        portMUX_TYPE spinlock;

        void (*push)(ctrl_buffer_data_t *self, const ctrl_sensor_data_t *in_item);
        size_t (*pop_block)(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_array, size_t max);
        size_t (*pop_all)(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_array);
        bool (*get_latest)(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_item);
        size_t (*get_count)(ctrl_buffer_data_t *self);
        bool (*seek_tail)(ctrl_buffer_data_t *self, size_t offset_from_head);
        bool (*check_and_clear_overflow)(ctrl_buffer_data_t *self);
    };

    ctrl_buffer_data_t *ctrl_get_buffers(uint8_t channel);

    void ctrl_buffer_data_init(void);
    bool ctrl_data_get_update(uint8_t channel);
    bool ctrl_display_data_get_update(uint8_t channel);
    bool ctrl_adq_data_get_update(uint8_t channel);

    /* --- SET_POINT --- */
    void ctrl_set_setpoint_value(uint8_t channel, float value);
    void ctrl_set_setpoint_min(uint8_t channel, float min);
    void ctrl_set_setpoint_max(uint8_t channel, float max);
    float ctrl_get_setpoint_value(uint8_t channel);
    float ctrl_get_setpoint_min(uint8_t channel);
    float ctrl_get_setpoint_max(uint8_t channel);

    /* --- DUTY --- */
    void ctrl_set_duty_value(uint8_t channel, float value);
    void ctrl_set_duty_min(uint8_t channel, float min);
    void ctrl_set_duty_max(uint8_t channel, float max);
    float ctrl_get_duty_value(uint8_t channel);
    float ctrl_get_duty_min(uint8_t channel);
    float ctrl_get_duty_max(uint8_t channel);

    /* --- KP --- */
    void ctrl_set_kp_value(uint8_t channel, float value);
    void ctrl_set_kp_min(uint8_t channel, float min);
    void ctrl_set_kp_max(uint8_t channel, float max);
    float ctrl_get_kp_value(uint8_t channel);
    float ctrl_get_kp_min(uint8_t channel);
    float ctrl_get_kp_max(uint8_t channel);

    /* --- KI --- */
    void ctrl_set_ki_value(uint8_t channel, float value);
    void ctrl_set_ki_min(uint8_t channel, float min);
    void ctrl_set_ki_max(uint8_t channel, float max);
    float ctrl_get_ki_value(uint8_t channel);
    float ctrl_get_ki_min(uint8_t channel);
    float ctrl_get_ki_max(uint8_t channel);

    /* --- KD --- */
    void ctrl_set_kd_value(uint8_t channel, float value);
    void ctrl_set_kd_min(uint8_t channel, float min);
    void ctrl_set_kd_max(uint8_t channel, float max);
    float ctrl_get_kd_value(uint8_t channel);
    float ctrl_get_kd_min(uint8_t channel);
    float ctrl_get_kd_max(uint8_t channel);

    /* --- Estado de las salidas --- */
    void ctrl_set_output_state(uint8_t channel, bool state);
    bool ctrl_get_output_state(uint8_t channel);

    /* --- ESTADO Y TIPO DE CONTROLADOR --- */
    void ctrl_set_controller_type(uint8_t channel, ctrl_type_t type);
    ctrl_type_t ctrl_get_controller_type(uint8_t channel);

    /* --- Configuración de adquisición --- */
    void ctrl_set_adq_pretrigger_value(uint8_t channel, uint32_t pretrigger);
    void ctrl_set_adq_pretrigger_min(uint8_t channel, uint32_t min);
    void ctrl_set_adq_pretrigger_max(uint8_t channel, uint32_t max);
    uint32_t ctrl_get_adq_pretrigger_value(uint8_t ch);
    uint32_t ctrl_get_adq_pretrigger_min(uint8_t ch);
    uint32_t ctrl_get_adq_pretrigger_max(uint8_t ch);

    /* --- Puntos de adquisición --- */
    void ctrl_set_adq_points_value(uint8_t channel, uint32_t points);
    void ctrl_set_adq_points_min(uint8_t channel, uint32_t min);
    void ctrl_set_adq_points_max(uint8_t channel, uint32_t max);
    uint32_t ctrl_get_adq_points_value(uint8_t ch);
    uint32_t ctrl_get_adq_points_min(uint8_t ch);
    uint32_t ctrl_get_adq_points_max(uint8_t ch);

    /* --- Fuente de disparo de adquisición --- */
    void ctrl_set_adq_trigger_source_value(uint8_t channel, adq_trigger_source_t source);
    void ctrl_set_adq_trigger_source_min(uint8_t channel, adq_trigger_source_t min);
    void ctrl_set_adq_trigger_source_max(uint8_t channel, adq_trigger_source_t max);
    adq_trigger_source_t ctrl_get_adq_trigger_source_value(uint8_t ch);
    adq_trigger_source_t ctrl_get_adq_trigger_source_min(uint8_t ch);
    adq_trigger_source_t ctrl_get_adq_trigger_source_max(uint8_t ch);

    /* --- Condici{on de finalizacion de adquisición --- */
    void ctrl_set_adq_compete_condition_value(uint8_t channel, adq_complete_event_t complete_condition);
    void ctrl_set_adq_compete_condition_min(uint8_t channel, adq_complete_event_t min);
    void ctrl_set_adq_compete_condition_max(uint8_t channel, adq_complete_event_t max);
    adq_complete_event_t ctrl_get_adq_compete_condition_value(uint8_t ch);
    adq_complete_event_t ctrl_get_adq_compete_condition_min(uint8_t ch);
    adq_complete_event_t ctrl_get_adq_compete_condition_max(uint8_t ch);

    size_t ctrl_debug_data_buffer_count(ctrl_buffer_data_t *self);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_DATA_H_ */