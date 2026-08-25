#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "hmi_ui.h"

// --- Punteros globales para actualizar valores ---
lv_obj_t *label_speed_value;
lv_obj_t *label_duty_value;
lv_obj_t *label_set_p_value;
lv_obj_t *label_set_kp_value;
lv_obj_t *label_set_ki_value;
lv_obj_t *label_set_kd_value;
lv_obj_t *label_min_d_value;
lv_obj_t *label_max_d_value;
lv_obj_t *label_out_state;
lv_obj_t *label_controller_type;
lv_obj_t *label_encoder_type;
lv_obj_t *label_encoder_resolution;
lv_obj_t *label_com_iface;
lv_obj_t *label_remote_state;
lv_obj_t *label_error_state;

// --- Colores ---
#define COLOR_RED_OUT_ON lv_color_hex(0xFF5000)
#define COLOR_GREEN_OUT_OFF lv_color_hex(0x008800)
#define COLOR_GREEN_SPEED lv_color_hex(0x00FF00)

#define COLOR_BLACK lv_color_hex(0x000000)
#define COLOR_GREY lv_color_hex(0x888888)
#define COLOR_WHITE lv_color_hex(0xFFFFFF)

// --- Inclusión de fuentes externas Verdana ---
LV_FONT_DECLARE(verdana_pro_semi_bold_14_glyph);
LV_FONT_DECLARE(verdana_pro_semi_bold_16_glyph);
LV_FONT_DECLARE(verdana_pro_semi_bold_18_glyph);
LV_FONT_DECLARE(verdana_pro_semi_bold_20_glyph);
LV_FONT_DECLARE(verdana_pro_semi_bold_40_glyph);

void ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COLOR_BLACK, 0);
    lv_obj_set_style_text_color(scr, COLOR_WHITE, 0);

    /**************************************************************************
     * SECCIÓN 1: MENÚ LATERAL DERECHO
     *
     **************************************************************************/
    const char *menu_items[] = {"Set P", "Ctrl", "Set k", "Limit", ""};
    int menu_width = 72;
    int button_space = 5;
    int menu_height = (240 - 20 - button_space) / 5;

    for (int i = 0; i < 5; i++)
    {
        lv_obj_t *btn_box = lv_obj_create(scr);
        lv_obj_set_size(btn_box, menu_width, menu_height - button_space);
        lv_obj_set_pos(btn_box, 320 - menu_width, i * menu_height);

        lv_obj_set_style_radius(btn_box, 0, 0);

        lv_obj_set_style_bg_opa(btn_box, 0, 0);
        lv_obj_set_style_border_width(btn_box, 1, 0);
        lv_obj_set_style_bg_color(btn_box, COLOR_WHITE, 0);
        lv_obj_set_style_bg_opa(btn_box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(btn_box, COLOR_GREY, 0);
        // lv_obj_set_style_border_width(btn_box, 3, 0);
        lv_obj_set_style_border_side(btn_box, LV_BORDER_SIDE_NONE, 0);
        lv_obj_clear_flag(btn_box, LV_OBJ_FLAG_SCROLLABLE);

        if (menu_items[i][0] != '\0')
        {
            lv_obj_t *lbl = lv_label_create(btn_box);
            lv_label_set_text(lbl, menu_items[i]);
            lv_obj_set_style_text_font(lbl, &verdana_pro_semi_bold_16_glyph, 0);
            lv_obj_set_style_text_color(lbl, COLOR_BLACK, 0);
            // lv_obj_center(lbl);
        }
    }

    /**************************************************************************
     * SECCIÓN 2: BARRA INFERIOR DE ESTADO
     **************************************************************************/
    const char *status_items[] = {"PID", "QUAD", "2ppr", "USB", "", ""};
    int box_w = 40;
    int box_h = 16;
    int spacing = (320 - (6 * box_w)) / 7;

    lv_obj_t *bottom_line = lv_line_create(scr);
    static lv_point_precise_t bottom_line_pts[] = {{0, 220}, {320, 220}};
    lv_line_set_points(bottom_line, bottom_line_pts, 2);
    lv_obj_set_style_line_color(bottom_line, COLOR_WHITE, 0);
    lv_obj_set_style_line_width(bottom_line, 1, 0);

    // Usamos un arreglo de punteros a tus variables globales para actualizarlas directamente
    lv_obj_t **lbl_state_bar_refs[] = {
        &label_controller_type,
        &label_encoder_type,
        &label_encoder_resolution,
        &label_com_iface,
        &label_remote_state,
        &label_error_state};

    for (int i = 0; i < 6; i++)
    {
        lv_obj_t *stat_box = lv_obj_create(scr);
        lv_obj_set_size(stat_box, box_w, box_h);
        lv_obj_set_pos(stat_box, spacing + i * (box_w + spacing), 223);

        lv_obj_set_style_radius(stat_box, 0, 0);
        lv_obj_set_style_bg_opa(stat_box, 0, 0);
        lv_obj_set_style_border_width(stat_box, 1, 0);
        lv_obj_set_style_border_color(stat_box, COLOR_BLACK, 0);
        lv_obj_set_style_pad_all(stat_box, 0, 0);
        lv_obj_clear_flag(stat_box, LV_OBJ_FLAG_SCROLLABLE);

        // Creamos el label directamente dentro del stat_box y lo asignamos al puntero global
        *(lbl_state_bar_refs[i]) = lv_label_create(stat_box);
        lv_label_set_text(*(lbl_state_bar_refs[i]), status_items[i]);
        lv_obj_set_style_text_font(*(lbl_state_bar_refs[i]), &verdana_pro_semi_bold_16_glyph, 0);
        lv_obj_set_style_text_color(*(lbl_state_bar_refs[i]), COLOR_WHITE, 0);
        lv_obj_align(*(lbl_state_bar_refs[i]), LV_ALIGN_CENTER, 0, 0);
    }

    /**************************************************************************
     * SECCIÓN 3: ÁREA Medicion (Datos)
     *
     **************************************************************************/
    int line1_height = 6;  // titulo de duty / estado de salida
    int line2_height = 27; // medición de duty
    int line3_height = 57; // titulo de speed
    int line4_height = 78; // medición de speed

    // --- Fila 1: Duty title---
    lv_obj_t *lbl_duty_title = lv_label_create(scr);
    lv_label_set_text(lbl_duty_title, "Duty");
    lv_obj_set_style_text_font(lbl_duty_title, &verdana_pro_semi_bold_14_glyph, 0);
    lv_obj_set_pos(lbl_duty_title, 10, line1_height);

    // --- Fila 1: Out State---
    label_out_state = lv_label_create(scr);
    lv_label_set_text(label_out_state, "Out: OFF");
    lv_obj_set_style_text_font(label_out_state, &verdana_pro_semi_bold_18_glyph, 0);
    lv_obj_set_style_text_color(label_out_state, COLOR_GREEN_OUT_OFF, 0);
    lv_obj_set_pos(label_out_state, 165, line1_height);

    // // --- Fila 1: Out State ---
    // label_out_state = lv_label_create(scr);
    // lv_label_set_text(label_out_state, "OFF");
    // lv_obj_set_style_text_font(label_out_state, &verdana_pro_semi_bold_18_glyph, 0);
    // lv_obj_set_style_text_color(label_out_state, COLOR_RED_OUT, 0);
    // lv_obj_set_pos(label_out_state, 200, line1_height);

    // --- Fila 2: Duty value ---
    label_duty_value = lv_label_create(scr);
    lv_label_set_text(label_duty_value, "0000.0");
    lv_obj_set_style_text_font(label_duty_value, &verdana_pro_semi_bold_20_glyph, 0);
    lv_obj_set_style_text_color(label_duty_value, COLOR_WHITE, 0);
    lv_obj_set_pos(label_duty_value, 17, line2_height);

    // --- Fila 2: Duty unit ---
    lv_obj_t *lbl_duty_unit = lv_label_create(scr);
    lv_label_set_text(lbl_duty_unit, "%");
    lv_obj_set_style_text_font(lbl_duty_unit, &verdana_pro_semi_bold_16_glyph, 0);
    lv_obj_set_style_text_color(lbl_duty_unit, COLOR_WHITE, 0);
    lv_obj_set_pos(lbl_duty_unit, 90, line2_height);

    // --- Fila 3: Speed title---
    lv_obj_t *lbl_speed_title = lv_label_create(scr);
    lv_label_set_text(lbl_speed_title, "Speed");
    lv_obj_set_style_text_font(lbl_speed_title, &verdana_pro_semi_bold_14_glyph, 0);
    lv_obj_set_style_text_color(lbl_speed_title, COLOR_WHITE, 0);
    lv_obj_set_pos(lbl_speed_title, 10, line3_height);

    // --- Fila 4: Speed value---
    label_speed_value = lv_label_create(scr);
    lv_label_set_text(label_speed_value, "0000.0");
    lv_obj_set_style_text_font(label_speed_value, &verdana_pro_semi_bold_40_glyph, 0);
    lv_obj_set_style_text_color(label_speed_value, COLOR_GREEN_SPEED, 0);
    lv_obj_set_pos(label_speed_value, 17, line4_height);

    // --- Fila 4: Speed units------
    lv_obj_t *lbl_speed_unit = lv_label_create(scr);
    lv_label_set_text(lbl_speed_unit, "RPM");
    lv_obj_set_style_text_font(lbl_speed_unit, &verdana_pro_semi_bold_20_glyph, 0);
    lv_obj_set_style_text_color(lbl_speed_unit, COLOR_WHITE, 0);
    lv_obj_set_pos(lbl_speed_unit, 150, line4_height + 10);

    // --- Línea separadora horizontal media ---
    lv_obj_t *mid_line = lv_line_create(scr);
    static lv_point_precise_t mid_line_pts[] = {{10, 122}, {230, 122}};
    lv_line_set_points(mid_line, mid_line_pts, 2);
    lv_obj_set_style_line_color(mid_line, COLOR_GREY, 0);
    lv_obj_set_style_line_width(mid_line, 1, 0);

    /**************************************************************************
     * SECCIÓN 4: ÁREA Config info
     *
     **************************************************************************/
    int line5_height = 132; // set point / minimo
    int info_height = 21;

    const char *pid_title_item[] = {"set:", "kp:", "ki:", "kd:"};
    const char *pid_cfg_values[] = {"0000.0", "00.000", "00.000", "00.000"};
    label_set_p_value = lv_label_create(scr);
    label_set_kp_value = lv_label_create(scr);
    label_set_ki_value = lv_label_create(scr);
    label_set_kd_value = lv_label_create(scr);
    lv_obj_t *lbl_pid_cfg_values[] = {label_set_p_value,
                                      label_set_kp_value,
                                      label_set_ki_value,
                                      label_set_kd_value};

    for (int i = 0; i < 4; i++)
    {
        int y_height = line5_height + i * info_height;

        // Caja de la etiqueta
        lv_obj_t *title_box = lv_obj_create(scr);
        lv_obj_set_size(title_box, 40, info_height);
        lv_obj_set_pos(title_box, 6, y_height);

        lv_obj_set_style_radius(title_box, 0, 0);
        lv_obj_set_style_bg_opa(title_box, 0, 0);
        lv_obj_set_style_border_width(title_box, 0, 0);
        lv_obj_set_style_pad_all(title_box, 0, 0);
        lv_obj_clear_flag(title_box, LV_OBJ_FLAG_SCROLLABLE);
        // Etiqueta
        lv_obj_t *title = lv_label_create(title_box);
        lv_label_set_text(title, pid_title_item[i]);
        lv_obj_set_style_text_font(title, &verdana_pro_semi_bold_16_glyph, 0);
        lv_obj_set_style_text_color(title, COLOR_WHITE, 0);
        lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(title, LV_ALIGN_TOP_RIGHT, 0, 0);

        // valor
        lv_label_set_text(lbl_pid_cfg_values[i], pid_cfg_values[i]);
        lv_obj_set_style_text_font(lbl_pid_cfg_values[i], &verdana_pro_semi_bold_16_glyph, 0);
        lv_obj_set_style_text_color(lbl_pid_cfg_values[i], COLOR_WHITE, 0);
        lv_obj_set_style_text_align(lbl_pid_cfg_values[i], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_pos(lbl_pid_cfg_values[i], 56, y_height);
    }

    // --- output limits info
    const char *out_cfg_info_title[] = {"min", "max"};
    const char *out_cfg_info_value[] = {"000.0%", "100.0%"};
    label_max_d_value = lv_label_create(scr);
    label_min_d_value = lv_label_create(scr);
    lv_obj_t *lbl_out_cfg_value[] = {label_min_d_value,
                                     label_max_d_value};

    for (int i = 0; i < 2; i++)
    {
        int y_height = line5_height + i * info_height;

        // Caja de la etiqueta
        lv_obj_t *title_box = lv_obj_create(scr);
        lv_obj_set_size(title_box, 40, info_height);
        lv_obj_set_pos(title_box, 124, y_height);

        lv_obj_set_style_radius(title_box, 0, 0);
        lv_obj_set_style_bg_opa(title_box, 0, 0);
        lv_obj_set_style_border_width(title_box, 0, 0);
        lv_obj_set_style_pad_all(title_box, 0, 0);
        lv_obj_clear_flag(title_box, LV_OBJ_FLAG_SCROLLABLE);
        // Etiqueta
        lv_obj_t *title = lv_label_create(title_box);
        lv_label_set_text(title, out_cfg_info_title[i]);
        lv_obj_set_style_text_font(title, &verdana_pro_semi_bold_16_glyph, 0);
        lv_obj_set_style_text_color(title, COLOR_WHITE, 0);
        lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(title, LV_ALIGN_TOP_RIGHT, 0, 0);

        // valor
        lv_label_set_text(lbl_out_cfg_value[i], out_cfg_info_value[i]);
        lv_obj_set_style_text_font(lbl_out_cfg_value[i], &verdana_pro_semi_bold_16_glyph, 0);
        lv_obj_set_style_text_color(lbl_out_cfg_value[i], COLOR_WHITE, 0);
        lv_obj_set_style_text_align(lbl_out_cfg_value[i], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_pos(lbl_out_cfg_value[i], 168, y_height);
    }
}

static char buf_speed[7];
static char buf_duty[7];
static char buf_kp[7];
static char buf_ki[7];
static char buf_kd[7];
static char buf_sp[7];
static char buf_lim_min[7];
static char buf_lim_max[7];
static const char out_state_on[] = "OUT: ON";
static const char out_state_off[] = "OUT: OFF";
static const char bar_state_err[] = "ERR";
static const char bar_state_rem[] = "REM";
static const char str_empty[] = "";

// static const char units_rad_seg[] = "1/s";
// static const char units_rpm[] = "RPM";
// static const char units_hz[] = "Hz";

void ui_update_value_speed(float speed)
{
    snprintf(buf_speed, sizeof(buf_speed), "%06.1f", speed);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_speed_value, buf_speed);
        lvgl_port_unlock();
    }
}

void ui_update_value_duty(float duty)
{
    snprintf(buf_duty, sizeof(buf_duty), "%06.2f", duty);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_duty_value, buf_duty);
        lvgl_port_unlock();
    }
}

void ui_update_value_point(float sp)
{
    snprintf(buf_sp, sizeof(buf_sp), "%06.1f", sp);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_set_p_value, buf_sp);
        lvgl_port_unlock();
    }
}

void ui_update_value_kp(float kp)
{
    snprintf(buf_kp, sizeof(buf_kp), "%06.4f", kp);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_set_kp_value, buf_kp);
        lvgl_port_unlock();
    }
}

void ui_update_value_ki(float ki)
{
    snprintf(buf_ki, sizeof(buf_ki), "%06.4f", ki);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_set_ki_value, buf_ki);
        lvgl_port_unlock();
    }
}

void ui_update_value_kd(float kd)
{
    snprintf(buf_kd, sizeof(buf_kd), "%06.4f", kd);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_set_kd_value, buf_kd); // CORREGIDO: label_set_kd_value
        lvgl_port_unlock();
    }
}

void ui_update_value_min(float min)
{
    snprintf(buf_lim_min, sizeof(buf_lim_min), "%05.1f%%", min);
    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_min_d_value, buf_lim_min);
        lvgl_port_unlock();
    }
}

void ui_update_value_max(float max)
{
    snprintf(buf_lim_max, sizeof(buf_lim_max), "%05.1f%%", max);

    if (lvgl_port_lock(0))
    {
        lv_label_set_text(label_max_d_value, buf_lim_max);
        lvgl_port_unlock();
    }
}

void ui_update_out_state(bool state)
{
    if (state)
    {

        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_out_state, out_state_on);
            lv_obj_set_style_text_color(label_out_state, COLOR_RED_OUT_ON, 0);
            lvgl_port_unlock();
        }
    }
    else
    {

        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_out_state, out_state_off);
            lv_obj_set_style_text_color(label_out_state, COLOR_GREEN_OUT_OFF, 0);
            lvgl_port_unlock();
        }
    }
}

void ui_update_ctrl_type(ctrl_type_t type)
{
    if (type == CTRL_OPEN)
    {
        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_controller_type, "OPEN");
            lvgl_port_unlock();
        }
    }
    else if (type == CTRL_PID)
    {
        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_controller_type, "PID");
            lvgl_port_unlock();
        }
    }
}

void ui_update_error_mark(bool state)
{

    if (state)
    {
        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_error_state, bar_state_err);
            lvgl_port_unlock();
        }
    }
    else
    {
        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_error_state, str_empty);
            lvgl_port_unlock();
        }
    }
}

void ui_update_remote_mark(bool state)
{
    if (state)
    {
        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_remote_state, bar_state_rem);
            lvgl_port_unlock();
        }
    }
    else
    {
        if (lvgl_port_lock(0))
        {
            lv_label_set_text_static(label_remote_state, str_empty);
            lvgl_port_unlock();
        }
    }
}
