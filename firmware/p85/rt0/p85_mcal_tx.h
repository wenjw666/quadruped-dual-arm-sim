#ifndef P85_MCAL_TX_H
#define P85_MCAL_TX_H
#include "p85_runtime.h"
/* Optional BSP adapter. Requires verified MCAL headers and build macros. */
bool p85_mcal_send(void *context, uint32_t id, bool fd, bool brs,
                   const uint8_t *bytes, size_t count);
#endif
