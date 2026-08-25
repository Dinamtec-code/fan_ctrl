#ifndef CTRL_PID_H_
#define CTRL_PID_H_

#ifdef __cplusplus
extern "C"
{
#endif
    typedef enum PID_STATUS
    {
        CTRL_PID_STS_OK,
        CTRL_PID_STS_ERR
    } ctrl_pid_status_t;

    typedef struct PID_STRUCT
    {
        float kp;
        float ki;
        float kd;
        // state[0] = e[n], state[1] = e[n-1], state[2] = e[n-2], state[3] = u[n-1]
        float state[4];
        float kd_ts_prev; // Almacena Kd / \Delta{t}[n-1]
    } ctrl_pid_f32_t;

    ctrl_pid_status_t ctrl_pid_set_kp(float kp, uint8_t channel);
    ctrl_pid_status_t ctrl_pid_set_ki(float ki, uint8_t channel);
    ctrl_pid_status_t ctrl_pid_set_kd(float kd, uint8_t channel);
    ctrl_pid_status_t ctrl_pid_update_coeff(uint8_t channel);

    void ctrl_pid_state_reset(uint8_t channel);
    float dsp_pid_update(float error, float period_sec, uint8_t channel);
    float ctrl_get_pid_output(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_PID_H_ */