#ifndef CTRL_PID_H_
#define CTRL_PID_H_

#ifdef __cplusplus
extern "C"
{
#endif

void dsp_pid_init(float Kp, float Ki, float Kd, float period_ms);
float dsp_pid_update(float error);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_PID_H_ */