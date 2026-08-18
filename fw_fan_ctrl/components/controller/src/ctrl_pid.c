#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_dsp.h" // Cabecera principal de ESP-DSP

static const char *TAG = "PID_DSP_S3";

// El filtro requiere un buffer de estado para almacenar las muestras pasadas w[n-1] y w[n-2]
// La estructura Directa II de esp-dsp necesita obligatoriamente 2 floats por sección.
static float biquad_state[2] = {0.0f, 0.0f};
static float coeffs[5]; // Almacenará {b0, b1, b2, a1, a2}

void dsp_pid_init(float Kp, float Ki, float Kd, float period_ms)
{
    // 1. Inicializar la librería DSP (comprueba soporte de instrucciones de hardware)
    esp_err_t ret = dsps_biquad_init_f32();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar las funciones Biquad de ESP-DSP");
        return;
    }

    // 2. Calcular las ganancias corregidas por tiempo
    float A0 = Kp + Ki * (period_ms / 60000.0f) + Kd;
    float A1 = (-Kp) - (2.0f * Kd);
    float A2 = Kd;

    // 3. Mapear al arreglo de coeficientes esperado por dsps_biquad_f32
    // Formato estándar de ESP-DSP: coeffs = {b0, b1, b2, a1, a2}
    // Nota: El signo de a1 y a2 se invierte internamente en la ecuación de la librería
    coeffs[0] = A0;   // b0
    coeffs[1] = A1;   // b1
    coeffs[2] = A2;   // b2
    coeffs[3] = 1.0f;  // a1 (Dado que la ecuación del PID resta -u[n-1], y la lib resta a1*u[n-1], colocamos 1.0f)
    coeffs[4] = 0.0f;  // a2

    // 4. Limpiar el historial de estados pasados
    memset(biquad_state, 0, sizeof(biquad_state));
    
    ESP_LOGI(TAG, "PID optimizado para Xtensa LX7 inicializado con éxito.");
}

float dsp_pid_update(float error)
{
    float input_signal[1];
    float output_signal[1];
    
    input_signal[0] = error;

    // Ejecuta el cálculo utilizando las instrucciones SIMD del ESP32-S3
    // Parámetros: entrada, salida, nro_muestras (1), coeficientes, estado
    dsps_biquad_f32(input_signal, output_signal, 1, coeffs, biquad_state);

    return output_signal[0];
}
