#ifndef CTRL_PWM_H_
#define CTRL_PWM_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void pwm_init(void);
    void ctrl_pwm_update_duty(uint8_t channel, float duty);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_PWM_H_ */