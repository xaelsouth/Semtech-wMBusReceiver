#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "radio.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276.h"

#define FSK_FREQUENCY_CENTER        868950000
#define FSK_DATARATE                100000
#define FSK_FREQUENCY_DEVIATION     50000
#define FSK_BANDWIDTH               (2*FSK_FREQUENCY_DEVIATION + FSK_DATARATE)
#define FSK_AFC_BANDWIDTH           (2*FSK_FREQUENCY_DEVIATION + FSK_DATARATE)
#define FSK_PREAMBLE_LENGTH         3
#define FSK_FIX_LENGTH_PAYLOAD_ON   true

#if (FSK_BANDWIDTH > 250000)
#error "FSK_BANDWIDTH can't exceed 250000!"
#endif

extern void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

extern void SX1276OnDio0Irq( void *content );
extern void SX1276OnDio1Irq( void *content );
extern void SX1276OnDio2Irq( void *content );
extern void SX1276OnDio3Irq( void *content );
extern void SX1276OnDio4Irq( void *content );
extern void SX1276OnDio5Irq( void *content );

/*
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

extern volatile unsigned synch_cnt;
extern volatile unsigned rx_timeout_cnt;
extern volatile unsigned rx_error_cnt;
extern volatile unsigned tx_timeout_cnt;


THD_FUNCTION(SX1276Thread, arg) {

  (void)arg;

  SX1276HwInit(&SPID1);

  // Radio initialization
  RadioEvents.RxDone = OnRxDone;
  Sx1276Radio.Init( &RadioEvents );

  Sx1276Radio.SetChannel( FSK_FREQUENCY_CENTER );
  Sx1276Radio.SetDeviation( FSK_FREQUENCY_DEVIATION );

  Sx1276Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, false,
                                0, 0, false, true );

  Sx1276Radio.Rx( 0 ); // Continuous Rx

#if 0
  uint8_t setting[REG_VERSION + 1];
  SX1276ReadBuffer(REG_OPMODE, setting + REG_OPMODE, sizeof(setting) - REG_OPMODE);
#endif

  while (true) {
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

    /* Serving events.*/
    if (evt & EVT_RF_DIO0) {
      SX1276OnDio0Irq(NULL);
    }
    if (evt & EVT_RF_DIO1) {
      SX1276OnDio1Irq(NULL);
    }
    if (evt & EVT_RF_DIO2) {
      ++synch_cnt;
      SX1276OnDio2Irq(NULL);
    }
    if (evt & EVT_RF_DIO3) {
      SX1276OnDio3Irq(NULL);
    }
    if (evt & EVT_RF_DIO4) {
      SX1276OnDio4Irq(NULL);
    }
    if (evt & EVT_RF_DIO5) {
      SX1276OnDio5Irq(NULL);
    }
    if (evt & EVT_RF_RX_DONE) {
    }
    if (evt & EVT_RF_RX_TIMEOUT_SYNCWORD) {
    }
    if (evt & EVT_RF_RX_TIMEOUT) {
      ++rx_timeout_cnt;
      SX1276RestartFskRx();
    }
    if (evt & EVT_RF_RX_ERROR) {
      ++rx_error_cnt;
      SX1276RestartFskRx();
    }
    if (evt & EVT_RF_TX_TIMEOUT) {
      ++tx_timeout_cnt;
    }
  }
}
