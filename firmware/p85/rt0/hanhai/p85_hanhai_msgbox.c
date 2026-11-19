#include "p85_board.h"
#include "p85_bst_msgbox.h"
#include "ipc_app_common.h"
#include "ipc_lflist_siso.h"

int p85_hanhai_receive_once(uint32_t (*now_ms)(void)) {
    uint8_t handle;
    void *mapped;
    bst_msg_queue_t *queue;
    packet_object_t *packet;
    int result;
    if (!now_ms || !p85_bst_msgbox_endpoint(&handle, &mapped)) return -1;
    queue = (bst_msg_queue_t *)mapped;
    /* RTOS implementation waits on recv_comp, collates used_list and recycles
     * the PREVIOUS completed_pack_list. Do not manually free these nodes and
     * do not combine query_info with the raw/fast-path get_rwmsg API. */
    result = ipc_trans_layer_query_info(0x90u, handle, 0u, 0u);
    if (result != 0) return result; /* peer status change or transport error */
    packet = KERNEL_2_USER(queue->completed_pack_list.tail,
                          queue->k_addr, (uintptr_t)queue);
    while (packet) {
        lflist_node_t *node = KERNEL_2_USER(packet->packed_list.tail,
                                           queue->k_addr, (uintptr_t)queue);
        /* P85 wire messages fit exactly one 32-byte payload. Never accept a
         * fragment from a multi-frame packet as a separate motor command. */
        if (node && !node->next) {
            const msg_object_t *object = KERNEL_2_USER(node->data,
                                        queue->k_addr, (uintptr_t)queue);
            if (object) (void)p85_bst_msgbox_ingest(&object->msg, now_ms());
        }
        packet = KERNEL_2_USER(packet->node.next,
                               queue->k_addr, (uintptr_t)queue);
    }
    return 0;
}
