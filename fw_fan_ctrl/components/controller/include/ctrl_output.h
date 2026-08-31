#ifndef CTRL_OUTPUT_H_
#define CTRL_OUTPUT_H_

#include "stdbool.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ctrl_output_init(void);
    void ctrl_output_state(uint8_t channel, bool state);

#ifdef __cplusplus
}
#endif

#endif /* CRTL_OUTPUT_H_ */