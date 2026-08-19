#ifndef HMI_UI_H_
#define HMI_UI_H_

#include "ctrl_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ui_init(void);

    // Esta función se puedes llamar desde tu Tarea estática, asegurándote
    // de usar los mutex (lvgl_port_lock)
    void ui_update_value_speed(float speed);
    void ui_update_value_duty(float duty);
    void ui_update_value_point(float sp);
    void ui_update_value_kp(float kp);
    void ui_update_value_ki(float ki);
    void ui_update_value_kd(float kd);
    void ui_update_value_min(float min);
    void ui_update_value_max(float max);
    void ui_update_out_state(bool state);
    void ui_update_ctrl_type(ctrl_type_t type);
    void ui_update_error_mark(bool state);
    void ui_update_remote_mark(bool state);
    void ui_publish_update(void);

#ifdef __cplusplus
}
#endif
#endif /* HMI_UI_H_ */