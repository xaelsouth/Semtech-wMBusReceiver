#include <time.h>
#include "ch.h"
#include "hal.h"

time_t GetTimeUnixSec(RTCDriver *rtcp) {
  RTCDateTime timespec;
  struct tm tim;

  rtcGetTime(rtcp, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, &tim, NULL);
  return mktime(&tim);
}

void GetTimeTm(RTCDriver *rtcp, struct tm *timp) {
  RTCDateTime timespec;

  rtcGetTime(rtcp, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, timp, NULL);
}

void SetTimeUnixSec(RTCDriver *rtcp, time_t unix_time) {
  RTCDateTime timespec;
  struct tm tim;
  struct tm *canary;

  /* If the conversion is successful the function returns a pointer
     to the object the result was written into.*/
  canary = localtime_r(&unix_time, &tim);
  osalDbgCheck(&tim == canary);

  rtcConvertStructTmToDateTime(&tim, 0, &timespec);
  rtcSetTime(rtcp, &timespec);
}
