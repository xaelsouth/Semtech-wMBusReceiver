/*!
 * \file      sx1272mb2das-board.c
 *
 * \brief     Target board SX1272MB2DAS shield driver implementation
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
#include "ch.h"
#include "hal.h"
#include "timer.h"
#include "radio.h"
#include "board-config.h"
#include "sx1272.h"
#include "sx1272-board.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1272GetPaSelect( uint32_t channel );

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Sx1272Radio =
{
    SX1272Init,
    SX1272GetStatus,
    SX1272SetModem,
    SX1272SetChannel,
    SX1272IsChannelFree,
    SX1272Random,
    SX1272SetRxConfig,
    SX1272SetTxConfig,
    SX1272CheckRfFrequency,
    SX1272GetTimeOnAir,
    SX1272Send,
    SX1272SetSleep,
    SX1272SetStby,
    SX1272SetRx,
    SX1272StartCad,
    SX1272SetTxContinuousWave,
    SX1272ReadRssi,
    SX1272Write,
    SX1272Read,
    SX1272WriteBuffer,
    SX1272ReadBuffer,
    SX1272SetMaxPayloadLength,
    SX1272SetPublicNetwork,
    SX1272GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
    SX1272SetDeviation,
};


/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

void SX1272IoInit( void )
{
}

void SX1272IoIrqInit( DioIrqHandler **irqHandlers )
{
#if (SX1272_DIO0_IN_USE == 1)
    palEnableLineEvent(SX1272.DIO0, PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(SX1272.DIO0, irqHandlers[0], NULL);
#endif

    palEnableLineEvent(SX1272.DIO1, PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(SX1272.DIO1, irqHandlers[1], NULL);

    palEnableLineEvent(SX1272.DIO2, PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(SX1272.DIO2, irqHandlers[2], NULL);

#if (SX1272_DIO3_IN_USE == 1)
    palEnableLineEvent(SX1272.DIO3, PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(SX1272.DIO3, irqHandlers[3], NULL);
#endif

#if (SX1272_DIO4_IN_USE == 1)
    palEnableLineEvent(SX1272.DIO4, PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(SX1272.DIO4, irqHandlers[4], NULL);
#endif

#if (SX1272_DIO5_IN_USE == 1)
    palEnableLineEvent(SX1272.DIO5, PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(SX1272.DIO5, irqHandlers[5], NULL);    
#endif
}

void SX1272IoDeInit( void )
{
#if (SX1272_DIO0_IN_USE == 1)
    palDisableLineEvent(SX1272.DIO0);
#endif

    palDisableLineEvent(SX1272.DIO1);

    palDisableLineEvent(SX1272.DIO2);

#if (SX1272_DIO3_IN_USE == 1)
    palDisableLineEvent(SX1272.DIO3);
#endif

#if (SX1272_DIO4_IN_USE == 1)
    palDisableLineEvent(SX1272.DIO4);
#endif

#if (SX1272_DIO5_IN_USE == 1)
    palDisableLineEvent(SX1272.DIO5);
#endif
}

void SX1272IoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )
    GpioInit( &DbgPinTx, RADIO_DBG_PIN_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DbgPinRx, RADIO_DBG_PIN_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX1272IoTcxoInit( void )
{
    // No TCXO component available on this board design.
}

void SX1272SetBoardTcxo( uint8_t state )
{
    (void)state;
    // No TCXO component available on this board design.
#if 0
    if( state == true )
    {
        TCXO_ON( );
        chThdSleepMilliseconds( BOARD_TCXO_WAKEUP_TIME );
    }
    else
    {
        TCXO_OFF( );
    }
#endif
}

uint32_t SX1272GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1272Reset( void )
{
    // Enables the TCXO if available on the board design
    SX1272SetBoardTcxo( true );

    palSetLine(SX1272.Reset);
    chThdSleepMilliseconds(1);
    palClearLine(SX1272.Reset);
    chThdSleepMilliseconds(6);
}

void SX1272SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1272Read( REG_PACONFIG );
    paDac = SX1272Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1272GetPaSelect( SX1272.Settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1272Write( REG_PACONFIG, paConfig );
    SX1272Write( REG_PADAC, paDac );
}

static uint8_t SX1272GetPaSelect( uint32_t channel )
{
    (void)channel;
    return RF_PACONFIG_PASELECT_RFO;
}

void SX1272SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1272AntSwInit( );
        }
        else
        {
            SX1272AntSwDeInit( );
        }
    }
}

void SX1272AntSwInit( void )
{
#if 0
    GpioInit( &AntSwitch, RADIO_ANT_SWITCH, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
#endif
}

void SX1272AntSwDeInit( void )
{
#if 0
    GpioInit( &AntSwitch, RADIO_ANT_SWITCH, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX1272SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
#if 0
    case RFLR_OPMODE_TRANSMITTER:
        GpioWrite( &AntSwitch, 1 );
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        GpioWrite( &AntSwitch, 0 );
        break;
#endif
    }
}

bool SX1272CheckRfFrequency( uint32_t frequency )
{
    (void)frequency;
    // Implement check. Currently all frequencies are supported
    return true;
}

#if defined( USE_RADIO_DEBUG )
void SX1272DbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

void SX1272DbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif
