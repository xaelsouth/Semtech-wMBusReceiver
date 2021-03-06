/*!
 * \file      sx1272-board.h
 *
 * \brief     Target board SX1272 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __SX1272_BOARD_H__
#define __SX1272_BOARD_H__

#define SX1272_FIFO_SIZE               64    /* The FIFO size is fixed to 64 bytes. */
#define SX1272_FIFO_THRESHOLD_LEVEL    20    /* May not exceed FIFO_SIZE! */
#define SX1272_DIO0_IN_USE             0
#define SX1272_DIO3_IN_USE             0
#define SX1272_DIO4_IN_USE             0
#define SX1272_DIO5_IN_USE             0
#define SX1272_SYNC_WORD_PATTERN       0x5555543DULL

#if SX1272_FIFO_THRESHOLD_LEVEL > SX1272_FIFO_SIZE
#error "FIFO_THRESHOLD_LEVEL may not exceed FIFO_SIZE!"
#endif

/*!
 * \brief Radio hardware registers initialization definition
 *
 * \remark Can be automatically generated by the SX1272 GUI (not yet implemented)
 */
#define SX1272_RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    /* Improved sensitivity, highest gain */                                        { MODEM_FSK , REG_LNA                , 0x23 },\
    /* RestartRxWithPLLClock, AfcAutoOn, AGC auto on, PreambleDetect, AGC & AFC  */ { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    /* RSSI Offset, RSSI smoothing using 8 samples */                               { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    /* AfcAutoClearOn */                                                            { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    /* PreambleDetectorOn, PreambleDetectorSize = 3 bytes, 4 chip errros per bit tolerated */ { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    /* ClkOut OFF */                                                                { MODEM_FSK , REG_OSC                , 0x07 },\
    /* AutoRestartRxMod = wait for PLL to lock, PreamblePolarity = 0x55, Sync on, Size of the Sync Word = SyncSize + 1 = 2 */ { MODEM_FSK , REG_SYNCCONFIG         , 0xb3 },\
    /* MSB */                                                                       { MODEM_FSK , REG_SYNCVALUE1         , UINT8_C(SX1272_SYNC_WORD_PATTERN>>24) },\
    /* --- */                                                                       { MODEM_FSK , REG_SYNCVALUE2         , UINT8_C(SX1272_SYNC_WORD_PATTERN>>16) },\
    /* --- */                                                                       { MODEM_FSK , REG_SYNCVALUE3         , UINT8_C(SX1272_SYNC_WORD_PATTERN>>8) },\
    /* LSB */                                                                       { MODEM_FSK , REG_SYNCVALUE4         , UINT8_C(SX1272_SYNC_WORD_PATTERN>>0) },\
    /* Variable length, Whiteninig, CRC on, CrcAutoClearOff */                      { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    /* FIFO level = 20 bytes */                                                     { MODEM_FSK , REG_FIFOTHRESH         , SX1272_FIFO_THRESHOLD_LEVEL },\
    /* Temperature change threshold = 10�C */                                       { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    /*  */                                                                          { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    /* Temp. change */                                                              { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_DETECTOPTIMIZE  , 0x43 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}                                                 \

/*!
 * \brief Initializes the radio I/Os pins interface
 */
void SX1272IoInit( void );

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX1272IoIrqInit( DioIrqHandler **irqHandlers );

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU low power modes
 */
void SX1272IoDeInit( void );

/*!
 * \brief Initializes the TCXO power pin.
 */
void SX1272IoTcxoInit( void );

/*!
 * \brief Initializes the radio debug pins.
 */
void SX1272IoDbgInit( void );

/*!
 * \brief Resets the radio
 */
void SX1272Reset( void );

/*!
 * \brief Sets the radio output power.
 *
 * \param [IN] power Sets the RF output power
 */
void SX1272SetRfTxPower( int8_t power );

/*!
 * \brief Set the RF Switch I/Os pins in low power mode
 *
 * \param [IN] status enable or disable
 */
void SX1272SetAntSwLowPower( bool status );

/*!
 * \brief Initializes the RF Switch I/Os pins interface
 */
void SX1272AntSwInit( void );

/*!
 * \brief De-initializes the RF Switch I/Os pins interface
 *
 * \remark Needed to decrease the power consumption in MCU low power modes
 */
void SX1272AntSwDeInit( void );

/*!
 * \brief Controls the antenna switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] opMode Current radio operating mode
 */
void SX1272SetAntSw( uint8_t opMode );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool SX1272CheckRfFrequency( uint32_t frequency );

/*!
 * \brief Enables/disables the TCXO if available on board design.
 *
 * \param [IN] state TCXO enabled when true and disabled when false.
 */
void SX1272SetBoardTcxo( uint8_t state );

/*!
 * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * \retval time Board TCXO wakeup time in ms.
 */
uint32_t SX1272GetBoardTcxoWakeupTime( void );

/*!
 * \brief Writes new Tx debug pin state
 *
 * \param [IN] state Debug pin state
 */
void SX1272DbgPinTxWrite( uint8_t state );

/*!
 * \brief Writes new Rx debug pin state
 *
 * \param [IN] state Debug pin state
 */
void SX1272DbgPinRxWrite( uint8_t state );

/*!
 * Radio hardware and global parameters
 */
extern SX1272_t SX1272;

#endif // __SX1272_BOARD_H__
