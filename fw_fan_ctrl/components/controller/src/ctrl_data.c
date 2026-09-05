#include "ctrl_data.h"
#include "ctrl_sensor.h"
#include "ctrl_output.h"

// Memoria estática para los datos
static ctrl_sensor_data_t ctrl_sensor_data_buffer[CTRL_NUM_CHANNELS][SENSOR_BUFFER_SIZE];

/* ========================================================================== */
/*                          MÉTODOS DEL BUFFER (OOP)                          */
/* ========================================================================== */

static void ctrl_data_buffer_push(ctrl_buffer_data_t *self, const ctrl_sensor_data_t *in_item)
{
    portENTER_CRITICAL(&self->spinlock);

    self->data[self->head] = *in_item;
    self->head = (self->head + 1) % self->capacity;

    if (self->head == self->tail)
    {
        // Lleno: se descarta el más viejo
        self->tail = (self->tail + 1) % self->capacity;
        self->overflow_flag = true;
    }

    portEXIT_CRITICAL(&self->spinlock);
}

static size_t ctrl_data_buffer_pop_block(ctrl_buffer_data_t *self,
                                         ctrl_sensor_data_t *out_array,
                                         size_t max)
{
    portENTER_CRITICAL(&self->spinlock);

    size_t avail = (self->head - self->tail + self->capacity) % self->capacity;
    size_t n = (max < avail) ? max : avail;

    for (size_t i = 0; i < n; i++)
    {
        out_array[i] = self->data[self->tail];
        self->tail = (self->tail + 1) % self->capacity;
    }

    portEXIT_CRITICAL(&self->spinlock);
    return n;
}

static size_t ctrl_data_buffer_pop_all(ctrl_buffer_data_t *self,
                                       ctrl_sensor_data_t *out_array)
{

    return self->pop_block(self, out_array, self->capacity);
}

static bool ctrl_data_buffer_get_latest(ctrl_buffer_data_t *self,
                                        ctrl_sensor_data_t *out_item)
{
    bool has_data = false;
    portENTER_CRITICAL(&self->spinlock);

    if (self->head != self->tail)
    {
        // El último dato escrito está una posición antes de head
        size_t latest_idx = (self->head == 0) ? (self->capacity - 1) : (self->head - 1);
        *out_item = self->data[latest_idx];
        has_data = true;
    }

    portEXIT_CRITICAL(&self->spinlock);
    return has_data;
}

static size_t ctrl_data_buffer_get_count(ctrl_buffer_data_t *self)
{
    portENTER_CRITICAL(&self->spinlock);
    size_t count = (self->head - self->tail + self->capacity) % self->capacity;
    portEXIT_CRITICAL(&self->spinlock);
    return count;
}

static bool ctrl_data_buffer_seek_tail(ctrl_buffer_data_t *self, size_t offset_from_head)
{
    portENTER_CRITICAL(&self->spinlock);

    size_t occupancy = (self->head - self->tail + self->capacity) % self->capacity;

    if (offset_from_head > occupancy)
    {
        portEXIT_CRITICAL(&self->spinlock);
        return false;
    }

    self->tail = (self->capacity + self->head - offset_from_head) % self->capacity;
    self->overflow_flag = false;

    portEXIT_CRITICAL(&self->spinlock);
    return true;
}

size_t ctrl_debug_data_buffer_count(ctrl_buffer_data_t *self)
{
    return ctrl_data_buffer_get_count(self);
}

static bool ctrl_data_buffer_check_and_clear_overflow(ctrl_buffer_data_t *self)
{
    portENTER_CRITICAL(&self->spinlock);
    bool flag = self->overflow_flag;
    self->overflow_flag = false;
    portEXIT_CRITICAL(&self->spinlock);
    return flag;
}

/* ========================================================================== */
/*                     INICIALIZACIÓN DE LOS MANEJADORES                      */
/* ========================================================================== */

// Creamos un arreglo de buffers para acceder fácilmente mediante el índice del canal
ctrl_buffer_data_t ctrl_sensor_buffers[CTRL_NUM_CHANNELS];

void ctrl_buffer_data_init(void)
{
    for (uint8_t ch = 0; ch < CTRL_NUM_CHANNELS; ch++)
    {
        ctrl_sensor_buffers[ch].data = &ctrl_sensor_data_buffer[ch][0];
        ctrl_sensor_buffers[ch].capacity = SENSOR_BUFFER_SIZE;
        ctrl_sensor_buffers[ch].head = 0;
        ctrl_sensor_buffers[ch].tail = 0;
        ctrl_sensor_buffers[ch].overflow_flag = false;
        portMUX_INITIALIZE(&ctrl_sensor_buffers[ch].spinlock);

        ctrl_sensor_buffers[ch].push = ctrl_data_buffer_push;
        ctrl_sensor_buffers[ch].pop_block = ctrl_data_buffer_pop_block;
        ctrl_sensor_buffers[ch].pop_all = ctrl_data_buffer_pop_all;
        ctrl_sensor_buffers[ch].get_latest = ctrl_data_buffer_get_latest;
        ctrl_sensor_buffers[ch].get_count = ctrl_data_buffer_get_count;
        ctrl_sensor_buffers[ch].seek_tail = ctrl_data_buffer_seek_tail;
        ctrl_sensor_buffers[ch].check_and_clear_overflow = ctrl_data_buffer_check_and_clear_overflow;
    }
}

ctrl_buffer_data_t *ctrl_get_buffers(uint8_t channel)
{
    if (channel >= CTRL_NUM_CHANNELS)
    {
        return NULL;
    }
    return &(ctrl_sensor_buffers[channel]);
}
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

ctrl_param_t adq_pretrig[CTRL_NUM_CHANNELS] = {
    {.value = 10, .max = 100, .min = 0},
    {.value = 10, .max = 100, .min = 0}};

ctrl_param_t adq_points[CTRL_NUM_CHANNELS] = {
    {.value = 1000, .max = 5000, .min = 0},
    {.value = 1000, .max = 5000, .min = 0}};

ctrl_param_t adq_trig_cond[CTRL_NUM_CHANNELS] = {
    {.value = ADQ_TRIGGER_SETPOIT, .max = ADQ_TRIGGER_COUNT, .min = ADQ_TRIGGER_NONE},
    {.value = ADQ_TRIGGER_SETPOIT, .max = ADQ_TRIGGER_COUNT, .min = ADQ_TRIGGER_NONE}};

ctrl_param_t adq_stop_cond[CTRL_NUM_CHANNELS] = {
    {.value = ADQ_COMPLETE_POINTS, .max = ADQ_COMPLETE_COUT, .min = ADQ_COMPLETE_NONE},
    {.value = ADQ_COMPLETE_POINTS, .max = ADQ_COMPLETE_COUT, .min = ADQ_COMPLETE_NONE}};

static bool data_update[CTRL_NUM_CHANNELS] = {false};
static bool display_data_update[CTRL_NUM_CHANNELS] = {false};
static bool adq_data_update[CTRL_NUM_CHANNELS] = {false};

bool ctrl_data_get_update(uint8_t channel)
{
    bool data = data_update[channel];
    data_update[channel] = false;
    return data;
}

bool ctrl_display_data_get_update(uint8_t channel)
{
    bool data = display_data_update[channel];
    display_data_update[channel] = false;
    return data;
}

bool ctrl_adq_data_get_update(uint8_t channel)
{
    bool data = adq_data_update[channel];
    adq_data_update[channel] = false;
    return data;
}

static inline void set_data_update(uint8_t channel)
{
    display_data_update[channel] = true;
    data_update[channel] = true;
}

/* --- Generic Parameter Helpers --- */

static inline float clamp_f(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

static inline float clamp_ui(uint32_t val, uint32_t min, uint32_t max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

static inline void param_set_value_f(ctrl_param_t params[], uint8_t ch, float val)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return;
    }
    params[ch].value = clamp_f(val, params[ch].min, params[ch].max);
}

static inline void param_set_value_ui(ctrl_param_t params[], uint8_t ch, uint32_t val)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return;
    }
    params[ch].value = clamp_ui(val, params[ch].min, params[ch].max);
}

static inline void param_set_min_f(ctrl_param_t params[], uint8_t ch, float min)
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

static inline void param_set_min_ui(ctrl_param_t params[], uint8_t ch, uint32_t min)
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

static inline void param_set_max_f(ctrl_param_t params[], uint8_t ch, float max)
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

static inline void param_set_max_ui(ctrl_param_t params[], uint8_t ch, uint32_t max)
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

static inline float param_get_value_f(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return -1.0f;
    }
    return params[ch].value;
}

static inline uint32_t param_get_value_ui(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return 0xFFFFFFFF;
    }
    return params[ch].value;
}

static inline float param_get_min_f(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return -1.0f;
    }
    return params[ch].min;
}

static inline uint32_t param_get_min_ui(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return 0xFFFFFFFF;
    }
    return params[ch].min;
}

static inline float param_get_max_f(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return -1.0f;
    }
    return params[ch].max;
}

static inline uint32_t param_get_max_ui(const ctrl_param_t params[], uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return 0xFFFFFFFF;
    }
    return params[ch].max;
}

/* ========================================================================== */
/*                                  SET_POINT                                 */
/* ========================================================================== */

void ctrl_set_setpoint_value(uint8_t ch, float v)
{
    param_set_value_f(set_point, ch, v);
    set_data_update(ch);
}
void ctrl_set_setpoint_min(uint8_t ch, float m)
{
    param_set_min_f(set_point, ch, m);
    set_data_update(ch);
}
void ctrl_set_setpoint_max(uint8_t ch, float m)
{
    param_set_max_f(set_point, ch, m);
    set_data_update(ch);
}
float ctrl_get_setpoint_value(uint8_t ch) { return param_get_value_f(set_point, ch); }
float ctrl_get_setpoint_min(uint8_t ch) { return param_get_min_f(set_point, ch); }
float ctrl_get_setpoint_max(uint8_t ch) { return param_get_max_f(set_point, ch); }

/* ========================================================================== */
/*                                    DUTY                                    */
/* ========================================================================== */

void ctrl_set_duty_value(uint8_t ch, float v)
{
    param_set_value_f(duty, ch, v);
    set_data_update(ch);
}
void ctrl_set_duty_min(uint8_t ch, float m)
{
    param_set_min_f(duty, ch, m);
    set_data_update(ch);
}
void ctrl_set_duty_max(uint8_t ch, float m)
{
    param_set_max_f(duty, ch, m);
    set_data_update(ch);
}
float ctrl_get_duty_value(uint8_t ch) { return param_get_value_f(duty, ch); }
float ctrl_get_duty_min(uint8_t ch) { return param_get_min_f(duty, ch); }
float ctrl_get_duty_max(uint8_t ch) { return param_get_max_f(duty, ch); }

/* ========================================================================== */
/*                                     KP                                     */
/* ========================================================================== */

void ctrl_set_kp_value(uint8_t ch, float v)
{
    param_set_value_f(kp, ch, v);
    set_data_update(ch);
}
void ctrl_set_kp_min(uint8_t ch, float m)
{
    param_set_min_f(kp, ch, m);
    set_data_update(ch);
}
void ctrl_set_kp_max(uint8_t ch, float m)
{
    param_set_max_f(kp, ch, m);
    set_data_update(ch);
}
float ctrl_get_kp_value(uint8_t ch) { return param_get_value_f(kp, ch); }
float ctrl_get_kp_min(uint8_t ch) { return param_get_min_f(kp, ch); }
float ctrl_get_kp_max(uint8_t ch) { return param_get_max_f(kp, ch); }

/* ========================================================================== */
/*                                     KI                                     */
/* ========================================================================== */

void ctrl_set_ki_value(uint8_t ch, float v)
{
    param_set_value_f(ki, ch, v);
    set_data_update(ch);
}
void ctrl_set_ki_min(uint8_t ch, float m)
{
    param_set_min_f(ki, ch, m);
    set_data_update(ch);
}
void ctrl_set_ki_max(uint8_t ch, float m)
{
    param_set_max_f(ki, ch, m);
    set_data_update(ch);
}
float ctrl_get_ki_value(uint8_t ch) { return param_get_value_f(ki, ch); }
float ctrl_get_ki_min(uint8_t ch) { return param_get_min_f(ki, ch); }
float ctrl_get_ki_max(uint8_t ch) { return param_get_max_f(ki, ch); }

/* ========================================================================== */
/*                                     KD                                     */
/* ========================================================================== */

void ctrl_set_kd_value(uint8_t ch, float v)
{
    param_set_value_f(kd, ch, v);
    set_data_update(ch);
}
void ctrl_set_kd_min(uint8_t ch, float m)
{
    param_set_min_f(kd, ch, m);
    set_data_update(ch);
}
void ctrl_set_kd_max(uint8_t ch, float m)
{
    param_set_max_f(kd, ch, m);
    set_data_update(ch);
}
float ctrl_get_kd_value(uint8_t ch) { return param_get_value_f(kd, ch); }
float ctrl_get_kd_min(uint8_t ch) { return param_get_min_f(kd, ch); }
float ctrl_get_kd_max(uint8_t ch) { return param_get_max_f(kd, ch); }

/* ========================================================================== */
/*                        ESTADO Y TIPO DE CONTROLADOR                        */
/* ========================================================================== */

void ctrl_set_output_state(uint8_t ch, bool state)
{
    if (ch < CTRL_NUM_CHANNELS)
    {
        output_state[ch] = state;
    }
    set_data_update(ch);
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
    set_data_update(ch);
}

ctrl_type_t ctrl_get_controller_type(uint8_t ch)
{
    if (ch >= CTRL_NUM_CHANNELS)
    {
        return CTRL_OPEN;
    }
    return controller_type[ch];
}

/******************************************************************************/
/*                        Configuración de adquisición                        */
/******************************************************************************/
/* ========================================================================== */
/*                                     pretrigger                             */
/* ========================================================================== */
void ctrl_set_adq_pretrigger_value(uint8_t ch, uint32_t pretrigger)
{
    param_set_value_ui(adq_pretrig, ch, pretrigger);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_pretrigger_min(uint8_t ch, uint32_t min)
{
    param_set_min_ui(adq_pretrig, ch, min);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_pretrigger_max(uint8_t ch, uint32_t max)
{
    param_set_max_ui(adq_pretrig, ch, max);
    set_data_update(ch);
    adq_cfg_update(ch);
}

uint32_t ctrl_get_adq_pretrigger_value(uint8_t ch) { return param_get_value_ui(adq_pretrig, ch); }
uint32_t ctrl_get_adq_pretrigger_min(uint8_t ch) { return param_get_min_ui(adq_pretrig, ch); }
uint32_t ctrl_get_adq_pretrigger_max(uint8_t ch) { return param_get_max_ui(adq_pretrig, ch); }

/* ========================================================================== */
/*                                     points                                 */
/* ========================================================================== */
void ctrl_set_adq_points_value(uint8_t ch, uint32_t points)
{
    param_set_value_ui(adq_points, ch, points);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_points_min(uint8_t ch, uint32_t min)
{
    param_set_min_ui(adq_points, ch, min);
    set_data_update(ch);
    adq_data_update[ch] = true;
}

void ctrl_set_adq_points_max(uint8_t ch, uint32_t max)
{
    param_set_max_ui(adq_points, ch, max);
    set_data_update(ch);
    adq_cfg_update(ch);
}

uint32_t ctrl_get_adq_points_value(uint8_t ch) { return param_get_value_ui(adq_points, ch); }
uint32_t ctrl_get_adq_points_min(uint8_t ch) { return param_get_min_ui(adq_points, ch); }
uint32_t ctrl_get_adq_points_max(uint8_t ch) { return param_get_max_ui(adq_points, ch); }

/* ========================================================================== */
/*                             trigger source                                 */
/* ========================================================================== */
void ctrl_set_adq_trigger_source_value(uint8_t ch, adq_trigger_source_t source)
{
    param_set_value_ui(adq_trig_cond, ch, (uint32_t)source);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_trigger_source_min(uint8_t ch, adq_trigger_source_t min)
{
    param_set_value_ui(adq_trig_cond, ch, (uint32_t)min);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_trigger_source_max(uint8_t ch, adq_trigger_source_t max)
{
    param_set_value_ui(adq_trig_cond, ch, (uint32_t)max);
    set_data_update(ch);
    adq_cfg_update(ch);
}

adq_trigger_source_t ctrl_get_adq_trigger_source_value(uint8_t ch)
{
    return (adq_trigger_source_t)param_get_value_ui(adq_trig_cond, ch);
}

adq_trigger_source_t ctrl_get_adq_trigger_source_min(uint8_t ch)
{
    return (adq_trigger_source_t)param_get_min_ui(adq_trig_cond, ch);
}

adq_trigger_source_t ctrl_get_adq_trigger_source_max(uint8_t ch)
{
    return (adq_trigger_source_t)param_get_max_ui(adq_trig_cond, ch);
}

/* ========================================================================== */
/*                            complete condition                              */
/* ========================================================================== */
void ctrl_set_adq_compete_condition_value(uint8_t ch, adq_complete_event_t complete_condition)
{
    param_set_value_ui(adq_stop_cond, ch, (uint32_t)complete_condition);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_compete_condition_min(uint8_t ch, adq_complete_event_t min)
{
    param_set_value_ui(adq_stop_cond, ch, (uint32_t)min);
    set_data_update(ch);
    adq_cfg_update(ch);
}

void ctrl_set_adq_compete_condition_max(uint8_t ch, adq_complete_event_t max)
{
    param_set_value_ui(adq_stop_cond, ch, (uint32_t)max);
    set_data_update(ch);
    adq_cfg_update(ch);
}

adq_complete_event_t ctrl_get_adq_compete_condition_value(uint8_t ch)
{
    return (adq_complete_event_t)param_get_value_ui(adq_stop_cond, ch);
}

adq_complete_event_t ctrl_get_adq_compete_condition_min(uint8_t ch)
{
    return (adq_complete_event_t)param_get_min_ui(adq_stop_cond, ch);
}

adq_complete_event_t ctrl_get_adq_compete_condition_max(uint8_t ch)
{
    return (adq_complete_event_t)param_get_max_ui(adq_stop_cond, ch);
}
