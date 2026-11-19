#ifndef P85_R2_HANDOFF_H
#define P85_R2_HANDOFF_H
#include <stdint.h>
void p85_r2_handoff_init(void);
void p85_r2_handoff_started(uint32_t can2_result, uint32_t can3_result);
void p85_r2_handoff_tick(void);
#endif
