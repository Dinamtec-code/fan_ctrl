#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ctrl_data.h"
#include "ctrl_pid.h"
#include "ctrl_pwm.h"
#include "ctrl_task.h"

// 1. Definir el tamaño en BYTES (En ESP-IDF, el stack se mide en bytes, no en words)
#define CTRL_TASK_STACK_SIZE 4 * 1024

// 2. Reservar la memoria estáticamente (BSS) en la SRAM interna
// StackType_t en el port de ESP-IDF equivale a uint8_t
StaticTask_t x_ctrl_TaskBuffer;
StackType_t x_ctrl_TaskStack[CTRL_TASK_STACK_SIZE];
TaskHandle_t x_ctrl_TaskHandle = NULL;

void ctrl_init(void)
{
    x_ctrl_TaskHandle = xTaskCreateStaticPinnedToCore(
        ctrl_task,            // Puntero a la función de la tarea
        "Controller Task",    // Nombre (para depuración)
        CTRL_TASK_STACK_SIZE, // Tamaño de la pila
        NULL,                 // Parámetro de entrada
        3,                    // Prioridad
        x_ctrl_TaskStack,     // Arreglo estático para la pila
        &x_ctrl_TaskBuffer,   // Estructura estática para el TCB
        0                     // Anclado al Core 1 (APP_CPU)
    );
    pwm_init();
}

void ctrl_task(void *vpParameter)
{
    // Inicializa xLastWakeTime con el tiempo actual
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);

    while (1)
    {
        // Bloquea la tarea hasta el próximo ciclo de 1 ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        for (uint8_t i = 0; i < CTRL_NUM_CHANNELS; i++)
        {
            if (ctrl_get_output_state(i))
            {
                switch (ctrl_get_controller_type(i))
                {
                case CTRL_OPEN:
                    ctrl_set_duty_value(0, 15);
                    //                    ctrl_set_duty_value(i, ctrl_get_setpoint_value((uint8_t)i));
                    //                    ctrl_pwm_update_duty(ctrl_get_setpoint_value((uint8_t)i), i);
                    break;
                case CTRL_PID:
                    // ctrl_pwm_update_duty(1000, i);
                    ctrl_set_duty_value(0, 37);
                    break;
                default:
                    ctrl_set_duty_value(0, 50);
                }
            }
            else
            {
                ctrl_pwm_update_duty(0, i);
            }
        }
    }
}