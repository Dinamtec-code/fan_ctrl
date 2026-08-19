#ifndef CONTROLLER_DATA_H_
#define CONTROLLER_DATA_H_
#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SPEED_BUFFER_SIZE 4096
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