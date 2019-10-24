/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
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

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
    chVTObjectInit(&obj->Timer);
    obj->ReloadValue = 0;
    obj->Callback = callback;
    obj->Context = NULL;
}

void TimerSetContext( TimerEvent_t *obj, void* context )
{
    obj->Context = context;
}

void TimerStart( TimerEvent_t *obj )
{
    chVTSet(&obj->Timer, obj->ReloadValue, obj->Callback, obj->Context);
}

void TimerStop( TimerEvent_t *obj )
{
    chVTReset(&obj->Timer);
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    obj->ReloadValue = TIME_MS2I(value);
}

TimerTime_t TimerGetCurrentTime( void )
{
    RTCDateTime now;
    rtcGetTime(&RTCD1, &now);
    return  now.millisecond;
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
    if ( past == 0 )
    {
        return 0;
    }

    RTCDateTime now;
    rtcGetTime(&RTCD1, &now);

    if (now.millisecond > past)
      return now.millisecond - past;

    return (1u<<27) - past + now.millisecond;
}

