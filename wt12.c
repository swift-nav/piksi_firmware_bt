/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ch.h>
#include <stdio.h>
#include <string.h>
#include "peripherals/usart_chat.h"
#include "settings.h"
#include "nmea.h"

static const char WT12_MODULE[] = "wt12";

static struct {
  Mutex mutex;
  enum uart uart;
  bool needs_init;
} wt12_config;

static struct {
  /* Packet state */
  u8 rx_csum;
  u8 rxbuf[256];
  u8 rxptr;

  /* Connection state */
  bool connected;
  u16 ipod_transid;
  u16 acc_transid;
  u32 time;

  /* Locaiton Lingo state */
  u8 ctlData[2][8];
  char nmea_msg[90];
  Mutex nmea_msg_mutex;
} iap_state;

static void wt12_nmea_output(const char *s);

static WORKING_AREA_CCM(wa_iap_thread, 2048);
static msg_t iap_thread(void *arg);

static void iap_process_state(void);

/* This is notification from SBP thread on configuration change,
 * must resync with iAP thread here */
static bool wt12_config_uart_notify(struct setting *s, const char *val)
{
  chMtxLock(&wt12_config.mutex);
  if (wt12_config.uart == UART_NONE)
    usart_release(uart_state(wt12_config.uart));

  if (!s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    if (wt12_config.uart != UART_NONE)
      chMtxUnlock();
    return false;
  }

  if (wt12_config.uart == UART_NONE)
    goto done;

  wt12_config.needs_init = true;

  /* TODO configure iAP settings */
  /* TODO start iAP thread */
done:
  chMtxUnlock();
  return true;
}

void wt12_setup(void)
{
  nmea_dispatcher_register(wt12_nmea_output);
  chMtxInit(&wt12_config.mutex);
  SETTING_NOTIFY("bluetooth", "uart", wt12_config.uart, TYPE_UART, wt12_config_uart_notify);
  chThdCreateStatic(wa_iap_thread, sizeof(wa_iap_thread),
                    LOWPRIO, iap_thread, NULL);
}

static msg_t iap_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("bluetooth");

  while (true) {
    chMtxLock(&wt12_config.mutex);

    if (wt12_config.needs_init) {
      printf("Bluetooth initialising\n");
      usart_claim(uart_state(wt12_config.uart), WT12_MODULE);
      usart_escape(wt12_config.uart);
#define SENDWAIT(send, wait) \
      if (!usart_sendwait(wt12_config.uart, (send), (wait), 3000)) {\
        chMtxUnlock(); \
        usart_release(uart_state(wt12_config.uart)); \
        continue; \
      }
      SENDWAIT("AT\r", "OK\r\n");
      SENDWAIT("SET BT NAME Piksi\r", "\n");
      SENDWAIT("SET BT SSP 3 0\r", "\n");
      SENDWAIT("SET BT IDENT BT:55 f001 6.0.0 Piksi\r", "\n");
      SENDWAIT("SET PROFILE IAP PROD:\"Piksi\" MF:\"Swift Navigation\" "
               "MDL:\"xyz\" SN:\"PK0110\" "
               "PROTO:\"com.bluegiga.iwrap\" "
               "FW:0.8.1 HW:2.3.1 CAPS:0 LINGO:0E MODE0\r",
               "\n");
      SENDWAIT("RESET\r", "READY.\r\n");

      wt12_config.needs_init = false;
      printf("Bluetooth initialised, waiting for connection\n");
    }

    chMtxUnlock();
    if (wt12_config.needs_init)
      continue;
    chMtxLock(&wt12_config.mutex);

    /* Wait for 'IAP x OK xxxx xxxx\r\n' signalling connection from iPhone */
    if (usart_sendwait(wt12_config.uart, "", "IAP ", 1000)) {
      u8 buf[14];
      int link_id;
      u16 max_payload;
      usart_read_dma(&uart_state(wt12_config.uart)->rx, buf, sizeof(buf));
      //printf("Expecting connection, got '%s'\n", buf);
      if (sscanf((char*)buf, "%d OK %hX %hX",
                 &link_id, &iap_state.acc_transid, &max_payload) > 0) {
        /* Woohoo! We're connected! */
        printf("iAP Connected: %d, %x %x\n",
               link_id, iap_state.acc_transid, max_payload);
        memset(&iap_state, 0, sizeof(iap_state));
        chMtxInit(&iap_state.nmea_msg_mutex);
        iap_state.connected = true;
        while (iap_state.connected && !wt12_config.needs_init) {
          iap_process_state();
          /* Give a change for SBP to reconfigure us */
          chMtxUnlock();
          chMtxLock(&wt12_config.mutex);
        }
      }
    }

    chMtxUnlock();
    chThdSleepMilliseconds(100);
  }

  return 0;
}

#define IAP_START 0x55
#define LINGO_ID_LOCATION 0x0E
#define LINGO_LOCATION_AccessoryAck        0x00
#define LINGO_LOCATION_GetAccessoryCaps    0x01
#define LINGO_LOCATION_RetAccessoryCaps    0x02
#define LINGO_LOCATION_GetAccessoryControl 0x03
#define LINGO_LOCATION_RetAccessoryControl 0x04
#define LINGO_LOCATION_SetAccessoryControl 0x05
#define LINGO_LOCATION_GetAccessoryData    0x06
#define LINGO_LOCATION_RetAccessoryData    0x07
#define LINGO_LOCATION_SetAccessoryData    0x08
#define LINGO_LOCATION_AsyncAccessoryData  0x09
#define LINGO_LOCATION_iPodAck             0x80

static void iap_pkt_send(uint8_t lingo, uint8_t cmd, uint16_t transid,
		  const void *payload, uint16_t len)
{
  u8 buf[128];
  u8 i = 0;
  u8 tx_csum = 0;

  buf[i++] = IAP_START;
  buf[i++] = len + 4;
  buf[i++] = lingo;
  buf[i++] = cmd;
  buf[i++] = transid >> 8;
  buf[i++] = transid & 0xff;
  memcpy(&buf[i], payload, len);
  i += len;
  for (int j = 1; j < i; j++)
    tx_csum += buf[j];
  buf[i++] = -tx_csum;

  usart_write_dma(&uart_state(wt12_config.uart)->tx, buf, i);
}

static int iap_pkt_recv(u8 *lingo, u8 *cmd, u16 *transid, u8 **buf)
{
  static const u8 disconnect_str[] = "NO CARRIER";
  static u8 disconnect_i;
  u8 *rxbuf = iap_state.rxbuf;
  u8 c;
  if (usart_read_dma_timeout(&uart_state(wt12_config.uart)->rx, &c, 1, 100) != 1)
    return -1;

  /* If sequence 'NO CARRIER' comes outside of a packet, disconnect iAP */
  if ((iap_state.rxptr == 0) && (c == disconnect_str[disconnect_i])) {
    disconnect_i++;
    if (disconnect_str[disconnect_i] == 0) {
      printf("iAP disconnected\n");
      iap_state.connected = false;
      disconnect_i = 0;
      return -5;
    }
  }

  if ((iap_state.rxptr == 0) && (c != IAP_START))
    return -2;

  rxbuf[iap_state.rxptr++] = c;
  iap_state.rx_csum += c;

  if ((iap_state.rxptr < 2) || (iap_state.rxptr < iap_state.rxbuf[1] + 3))
    return -3; /* Haven't received a full packet yet */

  if (iap_state.rx_csum != IAP_START) {
    /* Discard packet */
    iap_state.rxptr = 0;
    iap_state.rx_csum = 0;
    return -4;
  }

  /* Valid packet! */
  *lingo = rxbuf[2];
  *cmd = rxbuf[3];
  *transid = (rxbuf[4] << 8) | rxbuf[5];
  *buf = &rxbuf[6];
  u8 ret = iap_state.rxptr - 7; /* Header + checksum is 7 bytes */
  iap_state.rxptr = 0;
  iap_state.rx_csum = 0;
  return ret;
}

static bool iap_process_pkt(void)
{
  u8 *payload;
  u8 lingo, cmd;
  int l;

  l = iap_pkt_recv(&lingo, &cmd, &iap_state.ipod_transid, &payload);

  /* If Lingo ID is not correct, exit. */
  if ((l < 0) || (lingo != LINGO_ID_LOCATION))
    return false;

  /* Process command */
  switch (cmd) {
  case LINGO_LOCATION_GetAccessoryCaps:
    switch (payload[0]) {
    case 0:
      /* Reply with syscaps */
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_RetAccessoryCaps, iap_state.ipod_transid,
                   "\0" "\1\0" "\0\0\0\0\0\0\0\4" "\0\0\0\0\0\0\0\3", 19);
      break;
    case 1:
      /* Reply with NMEA caps */
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_RetAccessoryCaps, iap_state.ipod_transid,
                   "\1" "\0\0\0\0\0\0\0\0", 9);
      break;
    default:
      /* Reply with bad parameter AccessoryAck */
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_AccessoryAck, iap_state.ipod_transid,
                   "\4\1", 2);
    }
    break;
  case LINGO_LOCATION_GetAccessoryControl:
    if (payload[0] < 2) {
      u8 buf[9] = {payload[0]};
      memcpy(&buf[1], iap_state.ctlData[payload[0]], 8);
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_RetAccessoryControl, iap_state.ipod_transid,
                   buf, 9);
    } else {
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_AccessoryAck, iap_state.ipod_transid,
                   "\4\3", 2);
    }
    break;
  case LINGO_LOCATION_SetAccessoryControl:
    if (payload[0] < 2) {
      memcpy(&iap_state.ctlData[payload[0]], &payload[1], 8);
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_AccessoryAck, iap_state.ipod_transid,
                   "\0\5", 2);
    } else {
      iap_pkt_send(LINGO_ID_LOCATION,
                   LINGO_LOCATION_AccessoryAck, iap_state.ipod_transid,
                   "\4\5", 2);
    }
    break;
  case LINGO_LOCATION_GetAccessoryData:
    printf("Not implemented! GetAccessoryData %d\n", payload[0]);
    break;
  case LINGO_LOCATION_SetAccessoryData:
    printf("SetAccessoryData %d - ignored\n", payload[0]);
    iap_pkt_send(LINGO_ID_LOCATION,
                 LINGO_LOCATION_AccessoryAck, iap_state.ipod_transid,
                 "\0\x8", 2);
    break;
  case LINGO_LOCATION_iPodAck:
    chMtxLock(&iap_state.nmea_msg_mutex);
    iap_state.nmea_msg[0] = 0;
    chMtxUnlock();
    iap_state.acc_transid++;
    break;
  }
  return true;
}

static void iap_process_state(void)
{
  if (!iap_process_pkt() && (chTimeElapsedSince(iap_state.time) < 1000))
    return;

  iap_state.time = chTimeNow();

  /* Run iAP state machine */
  /* XXX */
  chMtxLock(&iap_state.nmea_msg_mutex);
  if (iap_state.nmea_msg[0])
    iap_pkt_send(LINGO_ID_LOCATION,
                 LINGO_LOCATION_AsyncAccessoryData,
                 iap_state.acc_transid,
                 iap_state.nmea_msg, iap_state.nmea_msg[9] + 10);
  chMtxUnlock();
}

static void wt12_nmea_output(const char *s)
{
  if (!iap_state.connected || !(iap_state.ctlData[0][7] & 0x04))
    return;

  if (chMtxTryLock(&iap_state.nmea_msg_mutex)) {
    iap_state.nmea_msg[0] = 1;    /* Location service */
    iap_state.nmea_msg[1] = 0x80; /* NMEA 0183 sentence */
    memset(&iap_state.nmea_msg[2], 0, 8);
    iap_state.nmea_msg[9] = strlen(s);
    strncpy(&iap_state.nmea_msg[10], s, sizeof(iap_state.nmea_msg) - 10);
    chMtxUnlock();
  }
}

