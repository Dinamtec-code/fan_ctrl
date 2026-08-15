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

// --- Definición de Pines (Ajusta según tu placa) ---
#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLK_HZ (20 * 1000 * 1000) // 40 MHz
#define LCD_PIN_NUM_MISO -1
#define LCD_PIN_NUM_BK_LIGHT 13
#define LCD_PIN_NUM_CLK 12
#define LCD_PIN_NUM_MOSI 11

#define LCD_PIN_NUM_DC 14
#define LCD_PIN_NUM_RST 9
#define LCD_PIN_NUM_CS 10

// 1. Definir el tamaño en BYTES (En ESP-IDF, el stack se mide en bytes, no en words)
#define DISPLAY_TASK_STACK_SIZE 16 * 1024

// 2. Reservar la memoria estáticamente (BSS) en la SRAM interna
// StackType_t en el port de ESP-IDF equivale a uint8_t
StaticTask_t x_ui_displayTaskBuffer;
StackType_t x_ui_displayStack[DISPLAY_TASK_STACK_SIZE];
TaskHandle_t x_ui_displayTaskHandle = NULL;

// Variables globales para la UI
lv_disp_t *lcd_disp = NULL;

void hmi_display_init(void)
{
    xTaskCreatePinnedToCore(
        display_task,
        "Display_Task",
        2 * 8192,
        NULL,
        5, // Prioridad 5 (Deja prioridades más altas para TinyUSB si es necesario)
        NULL,
        1 // Anclado al Core 1 (PRO_CPU es 0, APP_CPU es 1)
    );
}

// --- Tarea principal del Display ---
void display_task(void *pvParameter)
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

static float speed = 0;
static float dut = 0;

void ui_task(void *pvParameter)
{
    static bool out = false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1)
    {
        // 1. Haces tu lógica y cálculos normalmente
        speed += 10;
        dut += 2.4;

        if (dut > 100)
        {
            dut = 0.0;
            if (out == true)
            {
                out = false;
            }
            else
            {
                out = true;
            }
            ui_update_out_state(out);
        }
        if (speed > 8000)
        {
            speed = 0.0;
        }

        ui_update_values(speed, dut);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}