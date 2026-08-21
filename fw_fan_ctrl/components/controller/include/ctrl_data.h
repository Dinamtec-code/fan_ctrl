#ifndef CONTROLLER_DATA_H_
#define CONTROLLER_DATA_H_
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SENSOR_BUFFER_SIZE 4096
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

    // Estructura de un punto de dato completo
    typedef struct
    {
        float speed;
        uint64_t timestamp_us;
    } ctrl_sensor_data_t;

    // Declaración adelantada (Forward declaration) para usarla en los punteros a función
    typedef struct ctrl_buffer_data ctrl_buffer_data_t;

    typedef struct ctrl_buffer_data
    {
        ctrl_sensor_data_t *data;
        size_t fifo_head;      // Donde escribimos
        size_t fifo_tail;      // De donde leemos
        size_t fifo_count;     // Elementos actuales en el buffer
        portMUX_TYPE spinlock; // Spinlock independiente para este canal

        // Métodos (reciben un puntero a sí mismos "self")
        void (*push)(ctrl_buffer_data_t *self, float speed, uint64_t timestamp_us);
        bool (*pop)(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_data);
        bool (*get_latest)(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_data);
        size_t (*get_count)(ctrl_buffer_data_t *self);
    } ctrl_buffer_data_t;

    // Exponemos un arreglo de manejadores, uno para cada canal
    extern ctrl_buffer_data_t ctrl_sensor_buffers[CTRL_NUM_CHANNELS];

    bool ctrl_data_get_update(void);

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

    /* --- ESTADO Y TIPO DE CONTROLADOR --- */
    void ctrl_set_output_state(uint8_t channel, bool state);
    bool ctrl_get_output_state(uint8_t channel);

    void ctrl_set_controller_type(uint8_t channel, ctrl_type_t type);
    ctrl_type_t ctrl_get_controller_type(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /*CONTROLLER_DATA_H_*/