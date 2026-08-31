#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_ili9341.h"
#include "esp_log.h"
#include "lvgl.h"
#include "hmi_display.h"
#include "hmi_ui.h"
#include "esp_heap_caps.h"
#include "usb_tmc_process.h"
#include "ctrl_data.h"
#include "ctrl_sensor.h"
#include "math.h"

// --- Definición de Pines (Ajusta según tu placa) ---
#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLK_HZ (20 * 1000 * 1000) // 40 MHz
#define LCD_PIN_NUM_MISO -1                 // Display SDO      pin 9
#define LCD_PIN_NUM_BK_LIGHT 14             // Display LED      pin 8
#define LCD_PIN_NUM_CLK 13                  // Display SCK      pin 7
#define LCD_PIN_NUM_MOSI 12                 // Display SDI      pin 6
#define LCD_PIN_NUM_DC 11                   // Display DC       pin 5
#define LCD_PIN_NUM_RST 10                  // Display RESET    pin 4
#define LCD_PIN_NUM_CS 9                    // Display CS       pin 3
                                            // Display GND      pin 2
                                            // Display VCC      pin 1

// 1. Definir el tamaño en BYTES (En ESP-IDF, el stack se mide en bytes, no en words)
#define DISPLAY_TASK_STACK_SIZE 8 * 1024

// 2. Reservar la memoria estáticamente (BSS) en la SRAM interna
// StackType_t en el port de ESP-IDF equivale a uint8_t
StaticTask_t x_ui_displayTaskBuffer;
StackType_t x_ui_displayStack[DISPLAY_TASK_STACK_SIZE];
TaskHandle_t x_ui_displayTaskHandle = NULL;

// Variables globales para la UI
lv_disp_t *lcd_disp = NULL;

// --- Tarea principal del Display ---
void display_init(void)
{
    // 1. Encender la retroiluminación (Backlight)
    gpio_set_direction(LCD_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, 1);

    // 2. Configurar el Bus SPI con DMA
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_CLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320 * 240 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 3. Configurar la interfaz IO del LCD
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    // Pasar LCD_HOST directamente como SPI2_HOST
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 4. Inicializar el panel ILI9341
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .rgb_ele_order = COLOR_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .vendor_config = NULL,
        .flags.reset_active_high = 0};

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // 5. Inicializar el puerto de LVGL para ESP-IDF
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // 6. Añadir la pantalla a LVGL
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = 320 * 240 * sizeof(uint16_t) / 10,
        .double_buffer = true,
        .hres = 320,
        .vres = 240,
        .monochrome = false,
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        }

    };

    lcd_disp = lvgl_port_add_disp(&disp_cfg);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // 7. Inicializar los elementos gráficos bajo el mutex de LVGL Port
    if (lvgl_port_lock(0))
    {
        ui_init();

        lvgl_port_unlock();
    }
    // 3. Crear la tarea usando la versión estática anclada al Core 1
    x_ui_displayTaskHandle = xTaskCreateStaticPinnedToCore(
        ui_task,                 // Puntero a la función de la tarea
        "Display_UI",            // Nombre (para depuración)
        DISPLAY_TASK_STACK_SIZE, // Tamaño de la pila
        NULL,                    // Parámetro de entrada
        4,                       // Prioridad
        x_ui_displayStack,       // Arreglo estático para la pila
        &x_ui_displayTaskBuffer, // Estructura estática para el TCB
        1                        // Anclado al Core 1 (APP_CPU)
    );

    // 8. Una vez configurado el display, la tarea de inicialización puede destruirse.
    // esp_lvgl_port se encarga de ejecutar la tarea del handler de LVGL internamente.
    vTaskDelete(NULL);
}

void ui_task(void *pvParameter)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    while (1)
    {

        ui_update_value_speed(ctrl_get_filtred_speed(0));
        ui_update_value_duty(ctrl_get_duty_value(0));

        if (ctrl_dislplay_data_get_update(0))
        {
            ui_update_value_point(ctrl_get_setpoint_value(0));
            ui_update_value_kp(ctrl_get_kp_value(0));
            ui_update_value_ki(ctrl_get_ki_value(0));
            ui_update_value_kd(ctrl_get_kd_value(0));
            ui_update_value_min(ctrl_get_duty_min(0));
            ui_update_value_max(ctrl_get_duty_max(0));
            ui_update_out_state(ctrl_get_output_state(0));
            ui_update_ctrl_type(ctrl_get_controller_type(0));
            ui_update_error_mark(tmc_scpi_has_errors());
            ui_update_remote_mark(tmc_scpi_has_connected());
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}