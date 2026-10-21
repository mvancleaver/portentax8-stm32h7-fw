/* SPDX-License-Identifier: Apache-2.0
 *
 * can_rpmsg_bridge.h — Phase 07.3 Plan 03 patch 0024.
 *
 * Forward declarations for the cs8h7-can OpenAMP RPMsg bridge between
 * Linux userspace and the H7 FDCAN1 peripheral. The IMPLEMENTATIONS live
 * in src/rpc.c (next to the existing rp_endpoints[] array, the
 * new_service_cb dispatcher, and the OPENAMP_send/create_endpoint calls
 * for the existing `rpc` and `raw` endpoints — keeps the static endpoint
 * array internal to rpc.c). This header is consumed by:
 *
 *   - src/rpc.c            (defines both functions)
 *   - src/can_handler.c    (calls fdcan_rx_to_rpmsg() in the FDCAN1 RX
 *                           drain paths; preserves the SPI bridge enqueue
 *                           path untouched per Plan 7.3-03 R-A5).
 *
 * The new endpoint runs PARALLEL to the existing x8h7 SPI bridge endpoints
 * (rpc + raw); it does NOT replace them. Frames received from the FDCAN1
 * RX FIFO are still SPI-enqueued for the existing transport AND copied to
 * the RPMsg endpoint (when bound). TX frames from the RPMsg endpoint are
 * forwarded to FDCAN1 via the existing can.c TX path — no edits to the
 * core driver (Plan 7.3-03 truth: "FDCAN1 driver TX/RX path is structurally
 * unchanged"). HAL_FDCAN_Init/Start/Stop/DeInit are NOT touched.
 */

#ifndef CAN_RPMSG_BRIDGE_H
#define CAN_RPMSG_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TX helper: called by rpmsg_recv_can_callback (in rpc.c) when Linux
 * delivers a frame on the cs8h7-can endpoint. Wraps the existing can.c
 * `can_write()` against fdcan_1. Returns 0 on success, negative on TX
 * FIFO full / HAL error (mirrors can_write semantics). NOT called from
 * any ISR context.
 *
 * Parameters mirror can_write():
 *   id    — 11 or 29 bit CAN ID (with CAN_EFF_FLAG bit set if extended)
 *   dlc   — data byte count (0..64; encoded into FDCAN DLC by can_write)
 *   flags — X8H7_CAN_FLAG_FD | _BRS | _ESI bitfield (bit layout matches
 *           cs8h7_rpmsg_frame.h CS8H7_RPMSG_CAN_FLAG_* values 1:1).
 *   data  — payload, dlc bytes consumed; safe to pass shorter buffers
 *           only when dlc is a true compile-time constant (otherwise
 *           caller must ensure 64-byte buffer for max-DLC frames).
 */
int can_send_frame_from_rpmsg(uint32_t id, uint8_t dlc, uint8_t flags,
                              const uint8_t *data);

/* RX-direction hook: called from can_handler.c immediately after the
 * existing SPI-bridge enqueue_packet(...CAN_RX_FRAME...) call drains a
 * frame from FDCAN1 RX FIFO0. Packs the same (id, len, flags, data) into
 * a CS8H7_RPMSG_FRAME_TYPE_RX rpmsg_can_frame_t and calls OPENAMP_send on
 * the cs8h7-can endpoint IF the endpoint is bound from the Linux side.
 * If the endpoint is unbound (Linux-side connect() not yet called) or
 * OPENAMP_send fails, increments a static drop counter and returns —
 * does NOT block the FDCAN RX drain. The SPI bridge path is structurally
 * unaffected (Option A parallel deployment, Plan 7.3-03 R-A5 mitigation).
 *
 * Safe to call from the FDCAN1 RX FIFO0 IRQ context (HAL_FDCAN_RxFifo0Callback)
 * because OPENAMP_send is queue-based; however prefer calling from the
 * polling drain (can_handle_data) when both paths fire so the ISR stays
 * lightweight. Calling from BOTH paths is safe — duplicate frames cannot
 * be produced because each FDCAN1 frame is read out exactly once by
 * either path (the same can_read() call drains the FIFO).
 */
void fdcan_rx_to_rpmsg(uint32_t id, uint8_t len, uint8_t flags,
                       const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* CAN_RPMSG_BRIDGE_H */
