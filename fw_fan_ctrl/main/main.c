#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hmi_display.h"

void app_main(void)
{
    hmi_display_init();
}
