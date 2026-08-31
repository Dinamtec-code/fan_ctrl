#include "driver/gpio.h"

#include "ctrl_pwm.h"
#include "ctrl_output.h"


#define PIN_OUTPUT_1 47
#define PIN_OUTPUT_2 48

void ctrl_output_init(void)
{
    // 1. Configurar los GPIOs como salidas
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_OUTPUT_1) | (1ULL << PIN_OUTPUT_2), // Máscara para ambos pines
        .mode = GPIO_MODE_OUTPUT,                                        // Modo salida
        .pull_up_en = GPIO_PULLUP_DISABLE,                               // Desactivar pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                           // Desactivar pull-down
        .intr_type = GPIO_INTR_DISABLE                                   // Desactivar interrupciones
    };
    gpio_config(&io_conf); // Aplicar configuración
    pwm_init();
}

void ctrl_output_state(uint8_t channel, bool state)
{
    switch (channel)
    {
    case 0:
        gpio_set_level(PIN_OUTPUT_1, state);
        break;
    case 1:
        gpio_set_level(PIN_OUTPUT_2, state);
        break;
    default:
    }
}
