#ifndef P85_BST_MSGBOX_H
#define P85_BST_MSGBOX_H
#include "p85_runtime.h"
#include "ipc_trans_layer.h"
/* R0 glue built by tools/build_hanhai_sdk.py, not deployed by CMake.
 * Initialize after the vendor MsgBox transport; before starting P85 tasks.
 */
int p85_bst_msgbox_open(p85_runtime *runtime);
void p85_bst_msgbox_close(void);
/* Access the service-owned route for the matching SDK receive task. The queue
 * remains vendor-owned; this does not transfer lifetime or node ownership.
 * Receive task may read this after open; close only after producers stop.
 */
bool p85_bst_msgbox_endpoint(uint8_t *handle, void **queue);
/* Call with ONE completed 32-byte vendor message. The SDK receive task owns
 * and follows SDK query_info recycling; this function only copies its data.
 * hanhai/p85_hanhai_msgbox.c implements the reviewed R0 receive loop.
 */
bool p85_bst_msgbox_ingest(const rw_msg_t *message, uint32_t capture_ms);
/* Install as p85_port.reply; called only by the owner control task. */
bool p85_bst_msgbox_reply(void *context, uint32_t cookie,
                          const uint8_t *payload, size_t bytes);
#endif
