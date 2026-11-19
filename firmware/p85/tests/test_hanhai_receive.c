/* Uses official MsgBox structures with a fake transport. No hardware IO.
 * Exercise copy-before-recycle, multi-packet traversal and fragment rejection. */
#include "p85_board.h"
#include "p85_bst_msgbox.h"
#include "ipc_app_common.h"
#include <stdio.h>
#include <string.h>

static unsigned checks, failures, queries, recycled, sent, can_tx;
static int query_result;
static bool recycle_on_query;
static bst_msg_queue_t queue;
static packet_object_t packets[5];
static lflist_node_t nodes[6];
static msg_object_t objects[6];
static rw_msg_t replies[4];
#define CHECK(x) do { checks++; if (!(x)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #x); failures++; } } while (0)

int32_t ipc_trans_layer_stub_create_handle(uint8_t endid, uint8_t fid,
        uint8_t sid, uint8_t cid, uint8_t *handle) {
    CHECK(endid == 0x90 && fid == 0 && sid == 2 && cid == 0x90);
    *handle = 11; return 0;
}
int32_t ipc_trans_layer_register_method(uint8_t endid, uint8_t handle, uint8_t cmd) {
    CHECK(endid == 0x90 && handle == 11 && cmd == 0x85); return 0;
}
int32_t ipc_trans_layer_mmap_session(uint8_t endid, uint8_t handle, void **mapped) {
    CHECK(endid == 0x90 && handle == 11); *mapped = &queue; return 0;
}
int32_t ipc_trans_layer_unregister_method(uint8_t endid, uint8_t handle) {
    CHECK(endid == 0x90 && handle == 11); return 0;
}
int32_t ipc_trans_layer_destroy_handle(uint8_t endid, uint8_t handle) {
    CHECK(endid == 0x90 && handle == 11); return 0;
}
int32_t ipc_trans_layer_stub_send_reply_msg(uint8_t endid, uint8_t handle, serdes_t *s) {
    CHECK(endid == 0x90 && handle == 11);
    CHECK(sent < 4);
    if (sent < 4) replies[sent] = s->msg_pool[0];
    sent++; return 0;
}
int32_t ipc_trans_layer_query_info(uint8_t endid, uint8_t handle,
                                 uint32_t polls, uint8_t from_user) {
    CHECK(endid == 0x90 && handle == 11 && polls == 0 && from_user == 0);
    queries++;
    if (recycle_on_query) {
        /* The real SDK owns recycling. Poison storage to reveal retained
         * pointers, then present an empty completion list to the consumer. */
        memset(objects, 0xcc, sizeof(objects));
        memset(nodes, 0, sizeof(nodes));
        memset(packets, 0, sizeof(packets));
        queue.completed_pack_list.tail = NULL;
        queue.completed_pack_list.head = NULL;
        recycled++;
        recycle_on_query = false;
    }
    return query_result;
}
static uintptr_t lock(void *ctx) { (void)ctx; return 0; }
static void unlock(void *ctx, uintptr_t saved) { (void)ctx; (void)saved; }
static bool send_can(void *ctx, uint32_t id, bool fd, bool brs,
                     const uint8_t *data, size_t size) {
    (void)ctx; (void)id; (void)fd; (void)brs; (void)data; (void)size;
    can_tx++; return true;
}
static uint32_t clock_ms(void) { return 500; }
static void message(unsigned index, uint16_t sequence, uint8_t token) {
    rw_msg_t *m = &objects[index].msg;
    uint8_t *payload = (uint8_t *)m->payload;
    memset(m, 0, sizeof(*m));
    m->header.pid = 0x10; m->header.cid = 0x90;
    m->header.sid = 2; m->header.cmd = 0x85;
    m->header.typ = MSGBX_MSG_TYPE_METHOD; m->header.len = 4;
    m->header.is_eof = 1; m->header.tok = token;
    memcpy(payload, "P85C", 4); payload[4] = 1;
    p85_put_u16(payload + 6, sequence);
    nodes[index].data = &objects[index];
}
int main(void) {
    p85_runtime runtime;
    p85_config cfg;
    p85_port port = {NULL, lock, unlock, send_can, p85_bst_msgbox_reply};
    CHECK(p85_hanhai_receive_once(clock_ms) < 0);
    CHECK(queries == 0);
    p85_default_config(&cfg, 3);
    CHECK(p85_runtime_init(&runtime, &cfg, &port, 500));
    CHECK(p85_bst_msgbox_open(&runtime) == 0);
    queue.k_addr = (uintptr_t)&queue; /* POSIX mapping is identity in this test. */
    CHECK(p85_hanhai_receive_once(NULL) < 0);
    CHECK(queries == 0);

    message(0, 100, 0xa1); message(1, 999, 0xff);
    message(2, 999, 0xff); message(3, 101, 0xa2);
    packets[0].packed_list.tail = &nodes[0];
    packets[0].node.next = &packets[1].node;
    packets[1].packed_list.tail = &nodes[1]; nodes[1].next = &nodes[2];
    packets[1].node.next = &packets[2].node;
    packets[2].packed_list.tail = &nodes[3];
    packets[2].node.next = &packets[3].node; /* Empty packet must be skipped. */
    packets[3].node.next = &packets[4].node;
    packets[4].packed_list.tail = &nodes[4]; /* Null data must be skipped. */
    queue.completed_pack_list.tail = &packets[0].node;
    CHECK(p85_hanhai_receive_once(clock_ms) == 0);
    CHECK(queries == 1 && runtime.count == 2 && sent == 0);
    CHECK(objects[0].msg.header.tok == 0xa1 && nodes[0].data == &objects[0]);
    CHECK(queue.completed_pack_list.tail == &packets[0].node && recycled == 0);
    CHECK(runtime.queue[0].time_ms == 500 && runtime.queue[1].time_ms == 500);

    /* The next SDK query reclaims storage BEFORE control processes its copy. */
    recycle_on_query = true;
    CHECK(p85_hanhai_receive_once(clock_ms) == 0);
    CHECK(recycled == 1 && runtime.count == 2);
    p85_runtime_step(&runtime, 501);
    CHECK(runtime.count == 0 && sent == 2);
    CHECK(replies[0].header.tok == 0xa1 && replies[1].header.tok == 0xa2);
    CHECK(p85_get_u16((const uint8_t *)replies[0].payload + 6) == 100);
    CHECK(p85_get_u16((const uint8_t *)replies[1].payload + 6) == 101);
    CHECK(!memcmp(replies[0].payload, "P85R", 4));
    CHECK(runtime.control.state == P85_OFF && can_tx == 0);

    /* Status changes and errors must not ingest an old completed list. */
    message(0, 102, 0xa3); packets[0].packed_list.tail = &nodes[0];
    queue.completed_pack_list.tail = &packets[0].node;
    query_result = 1;
    CHECK(p85_hanhai_receive_once(clock_ms) == 1 && runtime.count == 0);
    query_result = -12;
    CHECK(p85_hanhai_receive_once(clock_ms) == -12 && runtime.count == 0);
    p85_bst_msgbox_close();
    CHECK(p85_hanhai_receive_once(clock_ms) < 0 && queries == 4);
    printf("P85 Hanhai receive: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
