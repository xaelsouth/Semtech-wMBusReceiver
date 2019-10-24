#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "radio.h"
#include "sx1272Regs-Fsk.h"
#include "sx1272.h"

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

extern void SX1272OnDio0Irq( void *content );
extern void SX1272OnDio1Irq( void *content );
extern void SX1272OnDio2Irq( void *content );
extern void SX1272OnDio3Irq( void *content );
extern void SX1272OnDio4Irq( void *content );
extern void SX1272OnDio5Irq( void *content );

extern void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

extern volatile unsigned synch_cnt;
extern volatile unsigned rx_timeout_cnt;
extern volatile unsigned rx_error_cnt;
extern volatile unsigned tx_timeout_cnt;


/*
 * For the T1 mode the bit rate is specified at 100 kpbs +- 10%.
 * For the C1 mode the bit rate is specified at 100 kpbs +- 2%.
 * With the highest bit rate at 110kpbs and FIFO_THRESHOLD_LEVEL = 16 the
 * time for IRQ handler execution should not exceed
 * MAX_IRQ_HANDLER_EXECUTION_TIME = 1/bitrate_highest * 8 * FIFO_THRESHOLD_LEVEL = 1163 us.
 *
 * Measured with an oscilloscope.
 * SyncAddr IRQ handler will be called by an event from inside a thread.
 * FifoLevel IRQ handler will be subsequently called by an event from inside a thread.
 *
 * SyncAddr IRQ triggered.
 * SyncAddr IRQ handler called after: 4uS
 * SyncAddr IRQ handler execution: 70uS
 * FifoLevel IRQ handler called after (first block): 1300uS
 * FifoLevel IRQ handler execution (first block): 140uS < MAX_EXEC_TIME -> ok!
 * FifoLevel IRQ handler called after (subsequent block): 1150uS
 * FifoLevel IRQ handler execution (subsequent block): 130uS < MAX_EXEC_TIME -> ok!
 */
THD_FUNCTION(SX1272Thread, arg) {

  (void)arg;

  SX1272HwInit(&SPID1);

  // Radio initialization
  RadioEvents.RxDone = OnRxDone;
  Sx1272Radio.Init( &RadioEvents );

  Sx1272Radio.SetChannel( FSK_FREQUENCY_CENTER );
  Sx1272Radio.SetDeviation( FSK_FREQUENCY_DEVIATION );

  Sx1272Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, false,
                                0, 0, false, true );

  Sx1272Radio.Rx( 0 ); // Continuous Rx

#if 0
  uint8_t setting[REG_VERSION + 1];
  SX1272ReadBuffer(REG_OPMODE, setting + REG_OPMODE, sizeof(setting) - REG_OPMODE);
#endif

  while (true) {
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

    /* Serving events.*/
    if (evt & EVT_RF_DIO0) {
      SX1272OnDio0Irq(NULL);
    }
    if (evt & EVT_RF_DIO1) {
      SX1272OnDio1Irq(NULL);
    }
    if (evt & EVT_RF_DIO2) {
      ++synch_cnt;
      SX1272OnDio2Irq(NULL);
    }
    if (evt & EVT_RF_DIO3) {
      SX1272OnDio3Irq(NULL);
    }
    if (evt & EVT_RF_DIO4) {
      SX1272OnDio4Irq(NULL);
    }
    if (evt & EVT_RF_DIO5) {
      SX1272OnDio5Irq(NULL);
    }
    if (evt & EVT_RF_RX_DONE) {
    }
    if (evt & EVT_RF_RX_TIMEOUT_SYNCWORD) {
    }
    if (evt & EVT_RF_RX_TIMEOUT) {
      ++rx_timeout_cnt;
      SX1272RestartFskRx();
    }
    if (evt & EVT_RF_RX_ERROR) {
      ++rx_error_cnt;
      SX1272RestartFskRx();
    }
    if (evt & EVT_RF_TX_TIMEOUT) {
      ++tx_timeout_cnt;
    }
  }
}
