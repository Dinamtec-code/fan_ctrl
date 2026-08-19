#ifndef CTRL_PWM_H_
#define CTRL_PWM_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void pwm_init(void);
    void update_duty(float duty, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_PWM_H_ */