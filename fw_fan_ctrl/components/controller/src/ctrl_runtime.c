#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ctrl_data.h"
#include "ctrl_pid.h"
#include "ctrl_pwm.h"
#include "ctrl_sensor.h"
#include "ctrl_runtime.h"
#include "ctrl_output.h"

const static char *TAG_CTRL = "ctrl_task";

void ctrl_fsm_runtime(uint8_t channel, float speed, float period_sec)
{
    switch (ctrl_get_controller_type(channel))
    {
    case CTRL_OPEN:
        // Refresco periódico del PWM en lazo abierto
        // ESP_LOGI(TAG_CTRL, "speed: %8f \t OPEN", speed);
        ctrl_pwm_update_duty(channel, ctrl_get_setpoint_value(channel));
        break;

    case CTRL_PID:
        // Si hay una marca de que los datos del control se actualizaron
        if (ctrl_data_get_update(channel))
        {
            ctrl_pid_update_coeff(channel);
        }

        float error = ctrl_get_setpoint_value(channel) - speed;
        float pwm = dsp_pid_update(channel, error, period_sec);

        ctrl_pwm_update_duty(channel, pwm);

        break;
    default:
        // ESP_LOGI(TAG_CTRL, "ctrl_get_controller_type(channel) desconocido");
    }
}

void ctrl_init(void)
{
    // Iniciar buffers para guardar los datos de velocidad
    ctrl_buffer_data_init();
    ctrl_output_init();
    ctrl_sensor_init();
}
