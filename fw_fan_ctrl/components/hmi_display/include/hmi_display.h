#ifndef HMI_DISPLAY_H_
#define HMI_DISPLAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

    void hmi_display_init(void);
    void display_task(void *pvParameter);
    void ui_task(void *pvParameter);

#ifdef __cplusplus
}
#endif

#endif /* HMI_DISPLAY_H_ */
