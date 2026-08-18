#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hmi_display.h"
#include "usb_tmc_init.h"
#include "usb_tmc_process.h"

// 1. Definir el tamaño en BYTES
#define DISPLAY_TASK_STACK_SIZE 8 * 1024

// 2. Reservar la memoria estáticamente (BSS) en la SRAM interna StackType_t en el port de ESP-IDF equivale a uint8_t
StaticTask_t x_sys_init_TaskBuffer;
StackType_t x_sys_init_Stack[DISPLAY_TASK_STACK_SIZE];
TaskHandle_t x_sys_init_TaskHandle = NULL;

void sys_init_task(void *pvParameter)
{
    display_init();
}

void app_main(void)
{
    scpi_engine_init();
    tmc_hal_init();

    // Creamos una tarea estatica para inicializar todos los modulos del sistema.
    xTaskCreatePinnedToCore(
        sys_init_task,
        "Sys_Init_Task",
        8192,
        NULL,
        5, // Prioridad 5 (Deja prioridades más altas para TinyUSB si es necesario)
        NULL,
        1 // Anclado al Core 1 (PRO_CPU es 0, APP_CPU es 1)
    );
}
