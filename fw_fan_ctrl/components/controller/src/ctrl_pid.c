#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "ctrl_pid.h"
#include "ctrl_data.h"

//static const char *TAG = "PID_ASYNC_S3";

typedef enum PID_STATUS
{
    CTRL_PID_STS_OK,
    CTRL_PID_STS_ERR
} ctrl_pid_status_t;

typedef struct PID_STRUCT
{
    float kp;
    float ki;
    float kd;
    // state[0] = e[n], state[1] = e[n-1], state[2] = e[n-2], state[3] = u[n-1]
    float state[4];
    float kd_ts_prev; // Almacena Kd / \Delta{t}[n-1]
} ctrl_pid_f32_t;

static ctrl_pid_f32_t pid_handle[2];

ctrl_pid_status_t ctrl_pid_set_kp(float kp, uint8_t channel)
{
    if (kp >= ctrl_get_kp_max(channel))
    {
        pid_handle[channel].kp = ctrl_get_kp_max(channel);
    }
    else if (kp <= ctrl_get_kp_min(channel))
    {
        pid_handle[channel].kp = ctrl_get_kp_min(channel);
    }
    else
    {
        pid_handle[channel].kp = kp;
    }
    return CTRL_PID_STS_OK;
}

ctrl_pid_status_t ctrl_pid_set_ki(float ki, uint8_t channel)
{
    if (ki >= ctrl_get_ki_max(channel))
    {
        pid_handle[channel].ki = ctrl_get_ki_max(channel);
    }
    else if (ki <= ctrl_get_ki_min(channel))
    {
        pid_handle[channel].ki = ctrl_get_ki_min(channel);
    }
    else
    {
        pid_handle[channel].ki = ki;
    }
    return CTRL_PID_STS_OK;
}

ctrl_pid_status_t ctrl_pid_set_kd(float kd, uint8_t channel)
{
    if (kd >= ctrl_get_kd_max(channel))
    {
        pid_handle[channel].kd = ctrl_get_kd_max(channel);
    }
    else if (kd <= ctrl_get_kd_min(channel))
    {
        pid_handle[channel].kd = ctrl_get_kd_min(channel);
    }
    else
    {
        pid_handle[channel].kd = kd;
    }
    return CTRL_PID_STS_OK;
}

ctrl_pid_status_t ctrl_pid_update_coeff(uint8_t channel)
{
    pid_handle[channel].kp = ctrl_get_kp_value(channel);
    pid_handle[channel].ki = ctrl_get_ki_value(channel);
    pid_handle[channel].kd = ctrl_get_kd_value(channel);
    return CTRL_PID_STS_OK;
}

void ctrl_pid_state_reset(uint8_t channel)
{
    memset(pid_handle[channel].state, 0, sizeof(pid_handle[channel].state));
    pid_handle[channel].kd_ts_prev = 0.0f;
}

float dsp_pid_update(uint8_t channel, float error, float period_sec)
{
    ctrl_pid_f32_t *pid = &pid_handle[channel];

    // Evitar división por cero si el evento asincrónico es instantáneo
    if (period_sec <= 0.000001f)
        period_sec = 0.000001f;

    // 1. Calcular Kd/Ts actual (Ts[n]) y Ki*Ts
    float kd_ts_current = pid->kd / period_sec;
    float ki_ts = pid->ki * period_sec;

    // 2. Coeficientes asincrónicos exactos
    float b0 = pid->kp + ki_ts + kd_ts_current;
    float b1 = -pid->kp - kd_ts_current - pid->kd_ts_prev;
    float b2 = pid->kd_ts_prev;

    // 3. Desplazamiento del historial de errores
    pid->state[2] = pid->state[1]; // e[n-2] = e[n-1]
    pid->state[1] = pid->state[0]; // e[n-1] = e[n]
    pid->state[0] = error;         // Nuevo e[n]

    // 4. Ecuación en diferencias
    float delta_u = (b0 * pid->state[0]) +
                    (b1 * pid->state[1]) +
                    (b2 * pid->state[2]);

    float output = pid->state[3] + delta_u; // u[n] = u[n-1] + delta_u

    // 5. Anti-Windup / Clamping
    float max_duty = ctrl_get_duty_max(channel);
    float min_duty = ctrl_get_duty_min(channel);

    if (output > max_duty)
    {
        output = max_duty;
    }
    else if (output < min_duty)
    {
        output = min_duty;
    }

    // 6. Actualizar estados para el próximo ciclo (n-1)
    pid->state[3] = output;
    pid->kd_ts_prev = kd_ts_current;
    // ESP_LOGI(TAG, "b0 %6f,\t b1: %6f,\t b2: %6f,\t kp: %6f,\t ki: %6f,\t kd: %6f,\t error: %6f", b0, b1, b2, pid->kp, pid->ki, pid->kd, error);
    return output;
}

float ctrl_get_pid_output(uint8_t channel)
{
    return pid_handle->state[3];
}
