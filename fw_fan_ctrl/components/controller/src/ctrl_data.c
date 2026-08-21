#include "ctrl_data.h"
#include "ctrl_sensor.h"

// Memoria estática para los datos
static ctrl_sensor_data_t ctrl_sensor_data_buffer[CTRL_NUM_CHANNELS][SENSOR_BUFFER_SIZE];

/* ========================================================================== */
/*                          MÉTODOS DEL BUFFER (OOP)                          */
/* ========================================================================== */

static void ctrl_data_buffer_push(ctrl_buffer_data_t *self, float speed, uint64_t timestamp_us)
{
    taskENTER_CRITICAL(&self->spinlock);

    self->data[self->fifo_head].speed = speed;
    self->data[self->fifo_head].timestamp_us = timestamp_us;

    self->fifo_head = (self->fifo_head + 1) % SENSOR_BUFFER_SIZE;

    if (self->fifo_count < SENSOR_BUFFER_SIZE)
    {
        self->fifo_count++;
    }
    else
    {
        // El buffer está lleno, sobrescribimos el más viejo y avanzamos la cola
        self->fifo_tail = (self->fifo_tail + 1) % SENSOR_BUFFER_SIZE;
    }

    taskEXIT_CRITICAL(&self->spinlock);
}

static bool ctrl_data_buffer_pop(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_data)
{
    bool has_data = false;
    taskENTER_CRITICAL(&self->spinlock);

    if (self->fifo_count > 0)
    {
        *out_data = self->data[self->fifo_tail];
        self->fifo_tail = (self->fifo_tail + 1) % SENSOR_BUFFER_SIZE;
        self->fifo_count--;
        has_data = true;
    }

    taskEXIT_CRITICAL(&self->spinlock);
    return has_data;
}

static bool ctrl_data_buffer_get_latest(ctrl_buffer_data_t *self, ctrl_sensor_data_t *out_data)
{
    bool has_data = false;
    taskENTER_CRITICAL(&self->spinlock);

    if (self->fifo_count > 0)
    {
        // El último dato escrito está una posición antes de fifo_head
        size_t latest_idx = (self->fifo_head == 0) ? (SENSOR_BUFFER_SIZE - 1) : (self->fifo_head - 1);
        *out_data = self->data[latest_idx];
        has_data = true;
    }

    taskEXIT_CRITICAL(&self->spinlock);
    return has_data;
}

static size_t ctrl_data_buffer_get_count(ctrl_buffer_data_t *self)
{
    size_t count = 0;
    taskENTER_CRITICAL(&self->spinlock);
    count = self->fifo_count;
    taskEXIT_CRITICAL(&self->spinlock);
    return count;
}

/* ========================================================================== */
/*                     INICIALIZACIÓN DE LOS MANEJADORES                      */
/* ========================================================================== */

// Creamos un arreglo de buffers para acceder fácilmente mediante el índice del canal
ctrl_buffer_data_t ctrl_sensor_buffers[CTRL_NUM_CHANNELS] = {
    [0] = {// primer canal
           .data = &ctrl_sensor_data_buffer[0][0],
           .fifo_head = 0,
           .fifo_tail = 0,
           .fifo_count = 0,
           .spinlock = portMUX_INITIALIZER_UNLOCKED,
           .push = ctrl_data_buffer_push,
           .pop = ctrl_data_buffer_pop,
           .get_latest = ctrl_data_buffer_get_latest,
           .get_count = ctrl_data_buffer_get_count},
    [1] = {// segundo canal
           .data = &ctrl_sensor_data_buffer[1][0],
           .fifo_head = 0,
           .fifo_tail = 0,
           .fifo_count = 0,
           .spinlock = portMUX_INITIALIZER_UNLOCKED,
           .push = ctrl_data_buffer_push,
           .pop = ctrl_data_buffer_pop,
           .get_latest = ctrl_data_buffer_get_latest,
           .get_count = ctrl_data_buffer_get_count} // fin de la inicializacion
};


static bool data_update = false;

/* --- Variables de estado y parámetros del controlador --- */

static bool output_state[CTRL_NUM_CHANNELS] = {false, false};

ctrl_type_t controller_type[CTRL_NUM_CHANNELS] = {CTRL_OPEN, CTRL_OPEN};

ctrl_param_t set_point[CTRL_NUM_CHANNELS] = {
    {.value = 01.0f, .max = 8000.0f, .min = 0.0f},
    {.value = 01.0f, .max = 8000.0f, .min = 0.0f}};

ctrl_param_t duty[CTRL_NUM_CHANNELS] = {
    {.value = 0.0f, .max = 100.0f, .min = 0.0f},
    {.value = 0.0f, .max = 100.0f, .min = 0.0f}};

ctrl_param_t kp[CTRL_NUM_CHANNELS] = {
    {.value = 0.0f, .max = 10.0f, .min = 0.0f},
    {.value = 0.0f, .max = 10.0f, .min = 0.0f}};

ctrl_param_t ki[CTRL_NUM_CHANNELS] = {
    {.value = 0.0f, .max = 10.0f, .min = 0.0f},
    {.value = 0.0f, .max = 10.0f, .min = 0.0f}};

ctrl_param_t kd[CTRL_NUM_CHANNELS] = {
    {.value = 0.0f, .max = 10.0f, .min = 0.0f},
    {.value = 0.0f, .max = 10.0f, .min = 0.0f}};

bool ctrl_data_get_update(void)
{
    bool data = data_update;
    data_update = false;
    return data;
}

static inline void set_data_update()
{
    data_update = true;
}

/* --- Generic Parameter Helpers --- */

static inline float clampf(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

static inline void param_set_value(ctrl_param_t params[], uint8_t ch, float val)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return;
    }
    params[ch].value = clampf(val, params[ch].min, params[ch].max);
}

static inline void param_set_min(ctrl_param_t params[], uint8_t ch, float min)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return;
    }
    params[ch].min = min;
    if (params[ch].value < params[ch].min)
    {
        params[ch].value = params[ch].min;
    }
}

static inline void param_set_max(ctrl_param_t params[], uint8_t ch, float max)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return;
    }
    params[ch].max = max;
    if (params[ch].value > params[ch].max)
    {
        params[ch].value = params[ch].max;
    }
}

static inline float param_get_value(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return -1.0f;
    }
    return params[ch].value;
}

static inline float param_get_min(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return -1.0f;
    }
    return params[ch].min;
}

static inline float param_get_max(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return -1.0f;
    }
    return params[ch].max;
}

/* ========================================================================== */
/*                                  SET_POINT                                 */
/* ========================================================================== */

void ctrl_set_setpoint_value(uint8_t ch, float v)
{
    param_set_value(set_point, ch, v);
    set_data_update();
}
void ctrl_set_setpoint_min(uint8_t ch, float m)
{
    param_set_min(set_point, ch, m);
    set_data_update();
}
void ctrl_set_setpoint_max(uint8_t ch, float m)
{
    param_set_max(set_point, ch, m);
    set_data_update();
}
float ctrl_get_setpoint_value(uint8_t ch) { return param_get_value(set_point, ch); }
float ctrl_get_setpoint_min(uint8_t ch) { return param_get_min(set_point, ch); }
float ctrl_get_setpoint_max(uint8_t ch) { return param_get_max(set_point, ch); }

/* ========================================================================== */
/*                                    DUTY                                    */
/* ========================================================================== */

void ctrl_set_duty_value(uint8_t ch, float v)
{
    param_set_value(duty, ch, v);
    set_data_update();
}
void ctrl_set_duty_min(uint8_t ch, float m)
{
    param_set_min(duty, ch, m);
    set_data_update();
}
void ctrl_set_duty_max(uint8_t ch, float m)
{
    param_set_max(duty, ch, m);
    set_data_update();
}
float ctrl_get_duty_value(uint8_t ch) { return param_get_value(duty, ch); }
float ctrl_get_duty_min(uint8_t ch) { return param_get_min(duty, ch); }
float ctrl_get_duty_max(uint8_t ch) { return param_get_max(duty, ch); }

/* ========================================================================== */
/*                                     KP                                     */
/* ========================================================================== */

void ctrl_set_kp_value(uint8_t ch, float v)
{
    param_set_value(kp, ch, v);
    set_data_update();
}
void ctrl_set_kp_min(uint8_t ch, float m)
{
    param_set_min(kp, ch, m);
    set_data_update();
}
void ctrl_set_kp_max(uint8_t ch, float m)
{
    param_set_max(kp, ch, m);
    set_data_update();
}
float ctrl_get_kp_value(uint8_t ch) { return param_get_value(kp, ch); }
float ctrl_get_kp_min(uint8_t ch) { return param_get_min(kp, ch); }
float ctrl_get_kp_max(uint8_t ch) { return param_get_max(kp, ch); }

/* ========================================================================== */
/*                                     KI                                     */
/* ========================================================================== */

void ctrl_set_ki_value(uint8_t ch, float v)
{
    param_set_value(ki, ch, v);
    set_data_update();
}
void ctrl_set_ki_min(uint8_t ch, float m)
{
    param_set_min(ki, ch, m);
    set_data_update();
}
void ctrl_set_ki_max(uint8_t ch, float m)
{
    param_set_max(ki, ch, m);
    set_data_update();
}
float ctrl_get_ki_value(uint8_t ch) { return param_get_value(ki, ch); }
float ctrl_get_ki_min(uint8_t ch) { return param_get_min(ki, ch); }
float ctrl_get_ki_max(uint8_t ch) { return param_get_max(ki, ch); }

/* ========================================================================== */
/*                                     KD                                     */
/* ========================================================================== */

void ctrl_set_kd_value(uint8_t ch, float v)
{
    param_set_value(kd, ch, v);
    set_data_update();
}
void ctrl_set_kd_min(uint8_t ch, float m)
{
    param_set_min(kd, ch, m);
    set_data_update();
}
void ctrl_set_kd_max(uint8_t ch, float m)
{
    param_set_max(kd, ch, m);
    set_data_update();
}
float ctrl_get_kd_value(uint8_t ch) { return param_get_value(kd, ch); }
float ctrl_get_kd_min(uint8_t ch) { return param_get_min(kd, ch); }
float ctrl_get_kd_max(uint8_t ch) { return param_get_max(kd, ch); }

/* ========================================================================== */
/*                        ESTADO Y TIPO DE CONTROLADOR                        */
/* ========================================================================== */

void ctrl_set_output_state(uint8_t ch, bool state)
{
    if (ch < CTRL_NUM_CHANNELS)
    {
        output_state[ch] = state;
    }
    set_data_update();
}

bool ctrl_get_output_state(uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return false;
    }
    return output_state[ch];
}

void ctrl_set_controller_type(uint8_t ch, ctrl_type_t t)
{
    if (ch < CTRL_NUM_CHANNELS)
    {
        controller_type[ch] = t;
    }
    set_data_update();
}

ctrl_type_t ctrl_get_controller_type(uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return CTRL_OPEN;
    }
    return controller_type[ch];
}