#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ctrl_data.h"
#include "ctrl_pid.h"
#include "ctrl_pwm.h"
#include "ctrl_sensor.h"
#include "ctrl_task.h"

// 1. Definir el tamaño en BYTES (En ESP-IDF, el stack se mide en bytes, no en words)
#define CTRL_TASK_STACK_SIZE 4 * 1024

// 2. Reservar la memoria estáticamente (BSS) en la SRAM interna
// StackType_t en el port de ESP-IDF equivale a uint8_t
StaticTask_t x_ctrl_TaskBuffer;
StackType_t x_ctrl_TaskStack[CTRL_TASK_STACK_SIZE];
TaskHandle_t x_ctrl_TaskHandle = NULL;

static void control_runtime(uint8_t channel, float speed)
{
    switch (ctrl_get_controller_type(channel))
    {
    case CTRL_OPEN:
        // Refresco periódico del PWM en lazo abierto
        ctrl_pwm_update_duty(ctrl_get_setpoint_value(channel), channel);
        break;

    case CTRL_PID:
        // Si get_sensor_speed detecta internamente el timeout por el timer_overflow, devolverá 0.0f
        if (speed == 0.0f)
        {
            // Acción de seguridad: forzar PWM a 0 o mínimo
            ctrl_pwm_update_duty(0, channel);
        }
        else
        {
            // Opcional: Ejecutar PID con la velocidad anterior o una calculada con el timestamp actual
        }
        break;
    }
}

void ctrl_init(void)
{
    x_ctrl_TaskHandle = xTaskCreateStaticPinnedToCore(
        ctrl_task,            // Puntero a la función de la tarea
        "Controller Task",    // Nombre (para depuración)
        CTRL_TASK_STACK_SIZE, // Tamaño de la pila
        NULL,                 // Parámetro de entrada
        10,                   // Prioridad
        x_ctrl_TaskStack,     // Arreglo estático para la pila
        &x_ctrl_TaskBuffer,   // Estructura estática para el TCB
        0                     // Anclado al Core 1 (APP_CPU)
    );
    pwm_init();
    ctrl_sensor_init();
}

void ctrl_task(void *vpParameter)
{
    uint32_t eventos_notificados = 0;

    while (1)
    {
        // Bloquea la tarea hasta recibir notificación O timeout
        BaseType_t xStatus = xTaskNotifyWait(0x00, 0xFFFFFFFF, &eventos_notificados, pdMS_TO_TICKS(50));

        if (xStatus == pdTRUE)
        {
            // === FLUJO 1: LA TAREA SE DESPERTÓ POR UNO O MÁS EVENTOS ===
            uint32_t eventos_pendientes = eventos_notificados;

            // Iterar SOLO por los bits que están en 1 (canales que notificaron)
            while (eventos_pendientes)
            {
                // Obtiene el índice del canal (0 para el bit 0, 1 para el bit 1, etc.)
                uint8_t i = __builtin_ctz(eventos_pendientes);

                // Limpia el bit actual para seguir con el próximo (si lo hubiera)
                eventos_pendientes &= ~(1UL << i);

                // Calcular velocidad
                float speed_rpm = ctrl_get_sensor_speed(i);
                // Actualizar ctrl_data
                // telemetry_buffer_push(i, speed_rpm);

                // Verificaciones de seguridad
                if (i < CTRL_NUM_CHANNELS && ctrl_get_output_state(i))
                {

                    // 3. Ejecutar algoritmo de control
                    control_runtime(i, speed_rpm);
                }
            }
        }
        else
        {
            // === FLUJO 2: TIMEOUT (Pasaron 50ms sin eventos de ningún canal) ===
            // Aquí sí se requiere un for para actualizar lazos abiertos o forzar apagados
            for (uint8_t i = 0; i < CTRL_NUM_CHANNELS; i++)
            {
                if (!ctrl_get_output_state(i))
                {
                    ctrl_pwm_update_duty(0, i);
                    continue;
                }
            }
        }
    }
}
