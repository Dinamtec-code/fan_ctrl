#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// #include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "driver/gpio.h" // Necesario en v6.0 para gpio_num_t
#include "esp_err.h"

#include "ctrl_pwm.h"
#include "ctrl_data.h"

// Definiciones para el modulo LEDC (PWM)
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE // OBLIGATORIO en ESP32-S3
#define PWM_OUTPUT_IO1 (35)
#define PWM_OUTPUT_IO2 (36)
#define PWM_CHANNEL1 LEDC_CHANNEL_0
#define PWM_CHANNEL2 LEDC_CHANNEL_1
#define PWM_DUTY_RES LEDC_TIMER_11_BIT
#define PWM_FREQUENCY (25000)

// Configuración del NCO
#define HW_RESOLUTION 11
#define EXTRA_BITS 4
#define TOTAL_BITS (HW_RESOLUTION + EXTRA_BITS) // 15 bits
#define MAX_DUTY_15 ((1UL << TOTAL_BITS) - 1)   // 32767
#define FRAC_MASK ((1UL << EXTRA_BITS) - 1)     // 0xF (15 en decimal)
#define FRAC_LIMIT (1UL << EXTRA_BITS)          // 16

void pwm_init(void)
{
    // 1. Configuración del temporario
    ledc_timer_config_t pwm_timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_DUTY_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    // 2. Configuración del canal
    ledc_channel_config_t pwm_channel1 = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL1,
        .timer_sel = PWM_TIMER,
        .gpio_num = PWM_OUTPUT_IO1,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel1));
    ledc_channel_config_t pwm_channel2 = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL2,
        .timer_sel = PWM_TIMER,
        .gpio_num = PWM_OUTPUT_IO2,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel2));
}

// Estado estático: Un acumulador de fracción INDEPENDIENTE para cada canal
// Asumiendo que usas canales 0 y 1. Si usas más, aumenta el tamaño a 8.
static uint32_t frac_acc[2] = {0, 0};

// Mapeo de canales lógicos (0, 1) a canales hardware del ESP32
// Ajusta PWM_CHANNEL1 y PWM_CHANNEL2 según tu definición (ej. LEDC_CHANNEL_0, LEDC_CHANNEL_1)
static const int hw_channels[] = {PWM_CHANNEL1, PWM_CHANNEL2};

void ctrl_pwm_update_duty(float duty, uint8_t channel)
{
    // Validaciones de seguridad
    if (channel >= 2)
        return; // Proteger contra índices fuera de rango del array

    ctrl_set_duty_value(channel, duty);

    // Conversión a resolución de 15 bits. EL duty lo tomamos del registo de datos que ya registró y acotó el valor.
    uint32_t target_15bit = (uint32_t)(ctrl_get_duty_value(channel) * (float)MAX_DUTY_15) / 100;

    // Separar parte entera (Hardware) y fraccionaria (Error)
    // La parte alta son los 11 bits reales
    uint32_t base_hw = target_15bit >> EXTRA_BITS;

    // La parte baja son los 4 bits de fracción
    uint32_t frac_part = target_15bit & FRAC_MASK;

    // Lógica NCO (Sigma-Delta de 1er orden)
    // Sumamos la fracción actual al acumulador histórico DE ESTE CANAL
    frac_acc[channel] += frac_part;

    uint32_t duty_final = base_hw;

    // El desborde por encima de extra bits el lo que pasa al duty real
    duty_final += (frac_acc[channel] >> EXTRA_BITS);
    frac_acc[channel] &= FRAC_MASK;

    // Saturación de seguridad (por si el duty fue 1.0 exacto y el acumulador empujó)
    if (duty_final > ((1 << HW_RESOLUTION) - 1))
    {
        duty_final = ((1 << HW_RESOLUTION) - 1);
    }

    // Aplicar al hardware LEDC
    // Usamos el array de mapeo para seleccionar el canal hardware correcto
    int hw_ch = hw_channels[channel];

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, hw_ch, duty_final));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, hw_ch));
}