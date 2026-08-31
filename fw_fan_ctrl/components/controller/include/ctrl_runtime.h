#ifndef CTRL_RUNTIME_H_
#define CTRL_RUNTIME_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ctrl_init();
    void ctrl_fsm_runtime(uint8_t channel, float speed, float period_sec);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_RUNTIME_H_ */