#include "ch.h"
#include "hal.h"

#if 0
/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend21(UARTDriver *uartp) {

  (void)uartp;
  chSysLockFromISR();
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend22(UARTDriver *uartp) {

  (void)uartp;
  chSysLockFromISR();
  palClearPad(GPIOB, GPIOB_RS485_DE);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr2(UARTDriver *uartp, uartflags_t e) {

  (void)uartp;
  (void)e;
  chSysLockFromISR();
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar2(UARTDriver *uartp, uint16_t c) {

  (void)uartp;
  (void)c;
  chSysLockFromISR();
  chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static const UARTConfig uart_cfg_2 = {
  txend21,
  txend22,
  rxend2,
  rxchar2,
  rxerr2,
  0,
  0,
  2400,
  0,
  0,
  0
};

static uint8_t uart_txbuf2[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB};
static uint8_t uart_rxbuf2[9] = {};


THD_FUNCTION(RS485Thread, arg) {

  (void)arg;

  chRegSetThreadName("RS485_control");

  /*
   * Activates UART driver 2
   */
  uartStart(&UARTD2, &uart_cfg_2);

  while(true) {
    uartStartSend(&UARTD2, sizeof(uart_txbuf2), uart_txbuf2);
    uartStartReceive(&UARTD2, sizeof(uart_rxbuf2), uart_rxbuf2);
    osDelay(5000);
    for (size_t i = 0; i < ARRAY_SIZE(uart_rxbuf2); i++) chprintf((BaseSequentialStream *)&SD2, "%02x", uart_rxbuf2[i]);
    chprintf((BaseSequentialStream *)&SD2, "\r\n");
  }
}
#endif
