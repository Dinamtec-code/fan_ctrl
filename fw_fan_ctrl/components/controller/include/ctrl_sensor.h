#ifndef CTRL_SENSOR_H_
#define CTRL_SENSOR_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Parámetros de la medición
#define CTRL_SENSOR_PPR 2U                 // Pulsos por revolución del encoder
#define CTRL_SENSOR_TIMER_RES_HZ 80000000U // Resolución del MCPWM Timer (ej. 80 MHz -> 1 tick = 12.5ns)
#define CTRL_SENSOR_TIMEOUT_TICKS (CTRL_SENSOR_TIMER_RES_HZ / 2)

    // Estructura optimizada para la ISR
    typedef struct
    {
        uint32_t last_edges[2];
    } capture_edges_t;

    // Datos compartidos entre ISR y Tarea
    typedef struct
    {
        volatile uint64_t timestamp_hw_us;
        volatile uint32_t period_ticks;
    } isr_to_task_data_t;

    typedef struct
    {
        float speed;
        uint64_t timestamp_us;
    } ctrl_sensor_data_t;

    // bool ctrl_make_sample(float speed, uint64_t timestamp_us, ctrl_sensor_data_t *out);

    float ctrl_get_sensor_speed(uint8_t channel);
    float ctrl_get_filtred_speed(uint8_t channel);
    void ctrl_sensor_init(void);

#ifdef __cplusplus
}
#endif

#endif