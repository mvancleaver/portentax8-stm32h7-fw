/*
 * Firmware for the Portenta X8 STM32H747AIIX/Cortex-M7 core.
 * Copyright (C) 2022 Arduino (http://www.arduino.cc/)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "openamp.h"
#include "arduino_openamp.h"
#include "stm32h7xx_hal.h"
#include "ringbuffer.h"
#include "stm32h7xx_ll_rcc.h"
#include "debug.h"

/* Phase 07.3 Plan 03 patch 0024 — cs8h7-can RPMsg endpoint additions.
 * - "can.h" — for can_write() (existing FDCAN1 TX path; Plan 7.3-03 truth
 *   "FDCAN1 driver TX/RX path is structurally unchanged" — this patch
 *   only CONSUMES the existing API, never edits HAL_FDCAN_Init/Start/Stop/DeInit).
 * - "can_rpmsg_bridge.h" — forward decls for the two helpers defined below.
 * - "cs8h7_rpmsg_frame.h" — shared 80-byte wire frame layout (lives outside
 *   the submodule at src/portenta_x8/include/; Makefile gains -I../include).
 */
#include "can.h"
#include "can_rpmsg_bridge.h"
#include "cs8h7_rpmsg_frame.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum endpoints_t {
  ENDPOINT_RAW = 0,
  ENDPOINT_RPC,
  /* Phase 07.3 Plan 03 patch 0024 — third endpoint, parallel to the existing
   * rpc + raw endpoints. Carries raw CAN-FD frames between Linux userspace and
   * the H7 FDCAN1 peripheral. Endpoint name "cs8h7-can". */
  ENDPOINT_CAN
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

/* Phase 07.3 Plan 03 patch 0024 — grew from [2] to [3] for the new ENDPOINT_CAN. */
static struct rpmsg_endpoint rp_endpoints[3];
extern ring_buffer_t virtual_uart_ring_buffer;
extern FDCAN_HandleTypeDef fdcan_1;

/* Phase 07.3 Plan 03 patch 0024 — diagnostic drop counters. Read via debug
 * trace channel if needed; not currently emitted on any endpoint. */
static volatile uint32_t cs8h7_rpmsg_rx_drops_unbound = 0;  /* OPENAMP_send before bind */
static volatile uint32_t cs8h7_rpmsg_rx_drops_send_err = 0; /* OPENAMP_send returned !=0 */
static volatile uint32_t cs8h7_rpmsg_tx_drops_bad_len = 0;  /* len != 80 from Linux */
static volatile uint32_t cs8h7_rpmsg_tx_drops_bad_type = 0; /* frame_type != TX */

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int rpmsg_recv_raw_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
  ring_buffer_queue_arr(&virtual_uart_ring_buffer, (const char *)data, len);

  return 0;
}

/* Phase 07.3 Plan 03 patch 0024 — cs8h7-can RPMsg endpoint callbacks + helpers.
 *
 * SAFETY: validate len AND frame_type before calling any FDCAN HAL function
 * (T-07.3.03-01 mitigation). The Linux side is trusted-by-OS-boundary
 * (chardev requires root or ophanim group) but bus-level protocol drift
 * still warrants drop-on-mismatch with diagnostic counters.
 *
 * NOTE: rpmsg_recv_can_callback runs in the OpenAMP RX work context, NOT
 * an ISR. fdcan_rx_to_rpmsg may run from EITHER ISR (HAL_FDCAN_RxFifo0Callback)
 * OR poll context (can_handle_data) — OPENAMP_send is queue-based and safe
 * from either. Both helpers are called by code in can_handler.c.
 */

int rpmsg_recv_can_callback(struct rpmsg_endpoint *ept, void *data,
                            size_t len, uint32_t src, void *priv)
{
  (void)ept; (void)src; (void)priv;

  /* T-07.3.03-01 — drop frames with unexpected size; the wire format is
   * fixed at 80 bytes (compile-asserted in cs8h7_rpmsg_frame.h). */
  if (len != sizeof(rpmsg_can_frame_t)) {
    cs8h7_rpmsg_tx_drops_bad_len++;
    return 0;
  }

  const rpmsg_can_frame_t *frame = (const rpmsg_can_frame_t *)data;

  /* T-07.3.03-01 — only TX frames are valid on the Linux→H7 path. RX/HELLO/
   * CYCLE_STATS frames here would be a protocol confusion; drop. */
  if (frame->frame_type != CS8H7_RPMSG_FRAME_TYPE_TX) {
    cs8h7_rpmsg_tx_drops_bad_type++;
    return 0;
  }

  /* DLC field carries the byte count in the wire frame (NOT the FDCAN DLC
   * encoding); can_send_frame_from_rpmsg passes it through to can_write
   * which performs the DLC->bytes encoding via its switch table. */
  return can_send_frame_from_rpmsg(frame->can_id, frame->dlc, frame->flags,
                                   frame->data);
}

/* TX helper — Linux→H7 direction. Wraps the existing can.c can_write()
 * against fdcan_1. No new FDCAN init/lifecycle code; only consumes the
 * existing TX FIFO submission path. */
int can_send_frame_from_rpmsg(uint32_t id, uint8_t dlc, uint8_t flags,
                              const uint8_t *data)
{
  return can_write(&fdcan_1, id, dlc, data, flags);
}

/* RX hook — H7→Linux direction. Called from can_handler.c after the
 * existing SPI bridge enqueue_packet(...CAN_RX_FRAME...) call drains a
 * frame from FDCAN1 RX FIFO0. Packs into rpmsg_can_frame_t and calls
 * OPENAMP_send on the cs8h7-can endpoint IFF bound from the Linux side.
 * On unbound endpoint OR send error: drop + increment counter; do NOT
 * block, do NOT panic. SPI bridge enqueue path is unaffected (Plan 7.3-03
 * R-A5 mitigation; Option A parallel deployment). */
void fdcan_rx_to_rpmsg(uint32_t id, uint8_t len, uint8_t flags,
                       const uint8_t *data)
{
  /* T-07.3.03-02 — endpoint may be unbound at boot (Linux side hasn't
   * called connect() yet). rdev==NULL is the OpenAMP "unbound" sentinel;
   * silently drop in that window so the FDCAN drain path is never blocked. */
  if (rp_endpoints[ENDPOINT_CAN].rdev == NULL) {
    cs8h7_rpmsg_rx_drops_unbound++;
    return;
  }

  rpmsg_can_frame_t frame;
  /* Zero-init padding + tail bytes beyond `len` so on-wire content is
   * deterministic (defensive; Linux side ignores beyond DLC anyway). */
  memset(&frame, 0, sizeof(frame));
  frame.can_id       = id;
  frame.flags        = flags;
  frame.dlc          = (len > CS8H7_RPMSG_CAN_DATA_MAX) ? CS8H7_RPMSG_CAN_DATA_MAX : len;
  frame.timestamp_us = HAL_GetTick() * 1000u;
  frame.frame_type   = CS8H7_RPMSG_FRAME_TYPE_RX;
  if (data != NULL && frame.dlc > 0) {
    memcpy(frame.data, data, frame.dlc);
  }

  int rc = OPENAMP_send(&rp_endpoints[ENDPOINT_CAN], &frame, sizeof(frame));
  if (rc < 0) {
    cs8h7_rpmsg_rx_drops_send_err++;
  }
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
  uint8_t buffer[1] = {0};
  struct rpmsg_endpoint *ept = NULL;
  rpmsg_ept_cb cb = rpmsg_recv_raw_callback;  /* default for rpc/raw */

  if (strcmp(name, "rpc") == 0) {
      ept = &rp_endpoints[ENDPOINT_RPC];
  } else if (strcmp(name, "raw") == 0) {
      ept = &rp_endpoints[ENDPOINT_RAW];
  } else if (strcmp(name, CS8H7_RPMSG_ENDPOINT_NAME) == 0) {
      /* Phase 07.3 Plan 03 patch 0024 — third endpoint, name "cs8h7-can".
       * Bound by the Linux-side CANdle-SDK RpmsgChar transport (Plan 7.3-04). */
      ept = &rp_endpoints[ENDPOINT_CAN];
      cb  = rpmsg_recv_can_callback;
  }

  if (ept) {
      OPENAMP_create_endpoint(ept, name, dest, cb, NULL);
      OPENAMP_send(ept, buffer, sizeof(buffer));
  }
}

int serial_rpc_begin() {

  /* Initialize OpenAmp and libmetal libraries */
  if (MX_OPENAMP_Init(RPMSG_HOST, new_service_cb) !=  HAL_OK) {
    return 0;
  }

  /* Initialize the rpmsg endpoint to set default addresses to RPMSG_ADDR_ANY */
  memset(rp_endpoints, 0, sizeof(rp_endpoints));

  return 1;
}

int serial_rpc_ready() {
  /*
  * The rpmsg service is initiate by the remote processor, on H7 new_service_cb
  * callback is received on service creation. Wait for the callback
  */
  uint32_t millis_start = HAL_GetTick();
  while (rp_endpoints[0].rdev == NULL || rp_endpoints[1].rdev == NULL) {
    if ((HAL_GetTick() - millis_start) >= 1000) {
      dbg_printf("M4 RPC timeout\n");
      return 0;
    }
  }

  return 1;
}

void serial_rpc_available() {
  OPENAMP_check_for_message();
}

void serial_rpc_write(uint8_t const * buf, size_t len) {
  // we'll only get rpc requests from "upstairs"
  OPENAMP_send(&rp_endpoints[ENDPOINT_RPC], buf, len);
}

void HSEM1_IRQHandler(void)
{
  OPENAMP_check_for_message();
  HAL_HSEM_IRQHandler();
}
