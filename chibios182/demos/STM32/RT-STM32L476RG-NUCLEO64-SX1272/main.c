/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "ch_test.h"
#include "cmsis_os.h"
#include "ff.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "radio.h"
#include "t1_c1_util.h"
#include "t1_c1_packet_decoder.h"

/*===========================================================================*/
/* Radio related.                                                            */
/*===========================================================================*/

#define DTG_RX_BUFFER_SIZE      512u

struct dtg_t
{
  uint8_t payload[DTG_RX_BUFFER_SIZE];
  size_t size;
  RTCDateTime timestamp;
  int16_t rssi;
  int8_t snr;
};

thread_t *sx1272_control_thread = NULL;
thread_t *sx1276_control_thread = NULL;

static THD_WORKING_AREA(waSX1272Thread, 4096);
THD_FUNCTION(SX1272Thread, arg);

static THD_WORKING_AREA(waSX1276Thread, 4096);
THD_FUNCTION(SX1276Thread, arg);

volatile unsigned synch_cnt = 0;
volatile unsigned rx_timeout_cnt = 0;
volatile unsigned rx_error_cnt = 0;
volatile unsigned tx_timeout_cnt = 0;

static MUTEX_DECL(statMtx);

static volatile struct RadioStatistics {
  unsigned total_datagrams;
  unsigned err_dtg_pool_too_small;
  unsigned crc_good_datagrams;
  unsigned crc_error_datagrams;
  unsigned err_3out6_datagrams, LorCfield_corected_dtgs;
  unsigned unrecognized_datagrams;
} radioStatistics;

#define DATAGRAM_POOL_SIZE 32
static struct dtg_t buffers[DATAGRAM_POOL_SIZE];

static msg_t free_buffers_queue[DATAGRAM_POOL_SIZE];
static mailbox_t free_buffers;

static msg_t filled_buffers_queue[DATAGRAM_POOL_SIZE];
static mailbox_t filled_buffers;

static volatile bool stream_dtgs_on = false;

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    struct dtg_t *dtg;
    msg_t msg;

    chMtxLock(&statMtx);
    radioStatistics.total_datagrams++;
    chMtxUnlock(&statMtx);

    msg = chMBFetchTimeout(&free_buffers, (msg_t *)&dtg, TIME_IMMEDIATE);

    if (msg == MSG_OK) {
      memcpy(dtg->payload, payload, size);
      rtcGetTime(&RTCD1, &dtg->timestamp);
      dtg->size = size;
      rssi = 0xFF; /** TODO: rssi */
      dtg->rssi = rssi;
      dtg->snr = snr;

      (void)chMBPostTimeout(&filled_buffers, (msg_t)dtg, TIME_INFINITE);
    }
    else if (msg == MSG_TIMEOUT) {
      chMtxLock(&statMtx);
      radioStatistics.err_dtg_pool_too_small++;
      chMtxUnlock(&statMtx);
    }
    else if (msg == MSG_RESET) {
      ;
    }
}

/*===========================================================================*/
/* RS485 related.                                                            */
/*===========================================================================*/

static THD_WORKING_AREA(waRS485Thread, 4096);
THD_FUNCTION(RS485Thread, arg);

thread_t *rs485_thread = NULL;

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
static FATFS SDC_FS;

/* FS mounted and ready.*/
static volatile bool fs_ready = false;

/* SD-Card is inserted.*/
static volatile bool sdc_inserted = false;

/* SDC driver configuration.*/
static const SDCConfig sdccfg = {.scratchpad = NULL, .bus_width = SDC_MODE_1BIT};

static char scan_files_path[FF_MAX_LFN] = "/"; /* Working variable for scan_files(). */

static FRESULT scan_files(BaseSequentialStream *chp, char *path) {
  FILINFO fno;
  FRESULT err;
  DIR dir;
  size_t i;
  char *fn;

  err = f_opendir(&dir, path);
  if (err != FR_OK) return err;

  i = strlen(path);
  while (((err = f_readdir(&dir, &fno)) == FR_OK) && fno.fname[0]) {
    if (FF_FS_RPATH && fno.fname[0] == '.')
      continue;
#ifdef _USE_LFN
    fn = *fno.lfname ? fno.lfname : fno.fname;
#else
    fn = fno.fname;
#endif
    if (fno.fattrib & AM_DIR) {
      *(path + i) = '/';
      strcpy(path + i + 1, fn);
      err = scan_files(chp, path);
      *(path + i) = '\0';
      if (err != FR_OK)
        break;
    }
    else {
      chprintf(chp, "%s/%s\r\n", path, fn);
    }
  }

  f_closedir(&dir);

  return err;
}

/*
 * Card insertion event.
 */
static void InsertHandler(void) {
  if (sdcConnect(&SDCD1))
    return;

  sdc_inserted = true;
}

/*
 * Card removal event.
 */
static void RemoveHandler(void) {

  sdcDisconnect(&SDCD1);
  sdc_inserted = false;
}

static void TestSdc(void) {
  FIL fil;
  UINT written_bytes;
  FRESULT err;
  BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */

  err = f_mkfs("0:", FM_FAT32, 0, work, sizeof(work));
  if (err) return;

  err = f_mount(&SDC_FS, "0:", 1);
  if (err != FR_OK) return;

  err = f_open(&fil, "0:/dtgs.txt", FA_WRITE | FA_CREATE_ALWAYS);
  if (err == FR_OK) {

    err = f_write(&fil, "test", 4, &written_bytes);
    err = f_sync(&fil);
    err = f_close(&fil);
  }

  err = f_mount(0, "0:", 0);
  if (err != FR_OK) return;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

thread_t *shell_thread = NULL;

static THD_WORKING_AREA(waShellThread, 4096);

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
    size_t n, total, largest;
    (void)argv;

    if (argc > 0) {
        chprintf(chp, "Usage: mem\r\n");
        return;
    }
    n = chHeapStatus(NULL, &total, &largest);
    //chprintf(chp, "core free memory  : %u bytes\r\n", chCoreStatus());
    chprintf(chp, "heap fragments    : %u\r\n", n);
    chprintf(chp, "heap free total   : %u bytes\r\n", total);
    chprintf(chp, "heap free largest : %u bytes\r\n", largest);
}

static void cmd_radio(BaseSequentialStream *chp, int argc, char *argv[]) {
  static RTCDateTime started;
  static RTCDateTime stopped;
  static struct RadioStatistics stat;
  char charbuf[1024];
  RTCDateTime now;

  if (argc != 1) {
    chprintf(chp, "Usage: radio on|off|stat\r\n");
    return;
  }

  if (strcmp(argv[0], "on") == 0) {
      rtcGetTime(&RTCD1, &started);
      chMtxLock(&statMtx);
      memset((void *)&radioStatistics, 0, sizeof(radioStatistics));
      chMtxUnlock(&statMtx);
      stream_dtgs_on = true;
  }
  else if (strcmp(argv[0], "off") == 0) {
    if (stream_dtgs_on) {
      stream_dtgs_on = false;
      chMtxLock(&statMtx);
      memcpy(&stat, (void *)&radioStatistics, sizeof(stat));
      chMtxUnlock(&statMtx);
      rtcGetTime(&RTCD1, &stopped);
    }
  }
  else if (strcmp(argv[0], "stat") == 0) {
    if (stream_dtgs_on) {
      rtcGetTime(&RTCD1, &now);
      chMtxLock(&statMtx);
      memcpy(&stat, (void *)&radioStatistics, sizeof(stat));
      chMtxUnlock(&statMtx);
    }
    else {
      memcpy(&now, &stopped, sizeof(now));
    }

    const time_t epoch = (now.millisecond - started.millisecond)/1000;
    const div_t total_dtgs_rxratio = div(stat.total_datagrams, epoch);
    const div_t good_dtgs_rxratio = div(stat.crc_good_datagrams, epoch);

    int charbuflen = 0;
    charbuflen += siprintf(charbuf+charbuflen, "rx error cnt: %u\r\n", rx_error_cnt);
    charbuflen += siprintf(charbuf+charbuflen, "rx timeout cnt: %u\r\n", tx_timeout_cnt);
    charbuflen += siprintf(charbuf+charbuflen, "err dtg pool too small: %u\r\n", stat.err_dtg_pool_too_small);
    charbuflen += siprintf(charbuf+charbuflen, "synch cnt: %u\r\n", synch_cnt);
    charbuflen += siprintf(charbuf+charbuflen, "total datagrams: %u\r\n", stat.total_datagrams);
    charbuflen += siprintf(charbuf+charbuflen, "crc good datagrams: %u\r\n", stat.crc_good_datagrams);
    charbuflen += siprintf(charbuf+charbuflen, "crc error datagrams: %u\r\n", stat.crc_error_datagrams);
    charbuflen += siprintf(charbuf+charbuflen, "unrecognized datagrams: %u\r\n", stat.unrecognized_datagrams);
    charbuflen += siprintf(charbuf+charbuflen, "err3outof6 datagrams: %u\r\n", stat.err_3out6_datagrams);
    charbuflen += siprintf(charbuf+charbuflen, "LorC field corrected dtgs: %u\r\n", stat.LorCfield_corected_dtgs);
    charbuflen += siprintf(charbuf+charbuflen, "total datagram rate: %u.%u\r\n", total_dtgs_rxratio.quot, total_dtgs_rxratio.rem);
    charbuflen += siprintf(charbuf+charbuflen, "crc good datagram rate: %u.%u\r\n", good_dtgs_rxratio.quot,good_dtgs_rxratio.rem);

    chprintf(chp, "%s", charbuf);
  }
  else
      chprintf(chp, "Usage: radio on|off|stat\r\n");
}

static void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]) {
  FRESULT err;
  (void)argc;
  (void)argv;

  if (!sdc_inserted) {
    chprintf(chp, "SDCard is not connected!\r\n");
    return;
  }

  if (fs_ready) {
    chprintf(chp, "SDCard is already mounted!\r\n");
    return;
  }

  err = f_mount(&SDC_FS, "0:", 1);
  if (err != FR_OK) {
    chprintf(chp, "Can't mount SDCard!\r\n");
    return;
  }

  chprintf(chp, "SDCard successfully mounted.\r\n");

  fs_ready = true;
}

static void cmd_umount(BaseSequentialStream *chp, int argc, char *argv[]) {
  FRESULT err;
  (void)argc;
  (void)argv;

  if (!sdc_inserted) {
    chprintf(chp, "SDCard is not connected!\r\n");
    return;
  }

  if (!fs_ready) {
    chprintf(chp, "SDCard is already unmounted!\r\n");
    return;
  }

  err = f_mount(0, "0:", 0);
  if (err != FR_OK) {
    chprintf(chp, "Can't unmount SDCard!\r\n");
    return;
  }

  chprintf(chp, "SDCard successfully unmounted.\r\n");

  fs_ready = false;
}

static void cmd_dir(BaseSequentialStream *chp, int argc, char *argv[]) {
  FILINFO fno;
  FRESULT res;
  DIR dir;
  char *fn;
  char path[FF_MAX_LFN] = "/";
  (void)argc;
  (void)argv;

  if (!sdc_inserted) {
    chprintf(chp, "SDCard is not connected!\r\n");
    return;
  }

  if (!fs_ready) {
    chprintf(chp, "SDCard is unmounted!\r\n");
    return;
  }

  res = f_opendir(&dir, path);
  if (res != FR_OK) {
    chprintf(chp, "Can't f_opendir!\r\n");
    return;
  }

  while (((res = f_readdir(&dir, &fno)) == FR_OK) && fno.fname[0]) {
    if (FF_FS_RPATH && fno.fname[0] == '.')
      continue;
#ifdef _USE_LFN
    fn = *fno.lfname ? fno.lfname : fno.fname;
#else
    fn = fno.fname;
#endif
    chprintf(chp, "%s/%s\r\n", path, fn);
  }

  if (res != FR_OK) {
    chprintf(chp, "Can't f_readdir!\r\n");
  }

  res = f_closedir(&dir);
  if (res != FR_OK) {
    chprintf(chp, "Can't f_closedir!\r\n");
  }
}

static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
  FIL fil;
  FRESULT err;
  UINT written_bytes;
  (void)argc;
  (void)argv;

  if (!sdc_inserted) {
    chprintf(chp, "SDCard is not connected!\r\n");
    return;
  }

  if (!fs_ready) {
    chprintf(chp, "SDCard is unmounted!\r\n");
    return;
  }

  err = f_unlink( "0:/dtgs.txt");
  if (err != FR_OK) {
    chprintf(chp, "Can't f_unlink!\r\n");
  }

  err = f_open(&fil, "0:/dtgs.txt", FA_WRITE | FA_OPEN_APPEND);
  //err = f_open(&fil, "0:/dtgs.txt", FA_WRITE | FA_CREATE_ALWAYS);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_open!\r\n");
    return;
  }

#if 0
  err = f_lseek(&fil, f_size(&fil));
  if (err != FR_OK) {
    chprintf(chp, "Can't f_lseek!\r\n");
  }
#endif

  err = f_write(&fil, "leckmich30", 10, &written_bytes);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_write!\r\n");
  }

#if 1
  err = f_sync(&fil);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_sync!\r\n");
  }
#endif

  err = f_close(&fil);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_close!\r\n");
  }
}

static void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]) {
  FIL fil;
  FRESULT err;
  UINT read_bytes;
  char buf[128] = {0};
  (void)argc;
  (void)argv;

  if (!sdc_inserted) {
    chprintf(chp, "SDCard is not connected!\r\n");
    return;
  }

  if (!fs_ready) {
    chprintf(chp, "SDCard is unmounted!\r\n");
    return;
  }

  err = f_open(&fil, "0:/dtgs.txt", FA_READ | FA_OPEN_EXISTING);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_open!\r\n");
    return;
  }

  err = f_read(&fil, buf, sizeof(buf), &read_bytes);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_read!\r\n");
  }
  else {
    chprintf(chp, "%s\r\n", buf);
  }

  err = f_close(&fil);
  if (err != FR_OK) {
    chprintf(chp, "Can't f_close!\r\n");
  }
}

static const ShellCommand commands[] = {
  {"radio", cmd_radio},
  {"mem", cmd_mem},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD2,
  commands
};


/*===========================================================================*/
/* Application related.                                                      */
/*===========================================================================*/

static thread_t *main_thread = NULL;

int main(void) {
  thread_descriptor_t shell_thd = {.name = "shell", .wbase = waShellThread,
                                   .wend = waShellThread + sizeof(waShellThread),
                                   .prio = HIGHPRIO + 1, .funcp = shellThread,
                                   .arg = (void *)&shell_cfg1};

  thread_descriptor_t sx1272_thd = {.name = "sx1272", .wbase = waSX1272Thread,
                                   .wend = waSX1272Thread + sizeof(waSX1272Thread),
                                   .prio = HIGHPRIO, .funcp = SX1272Thread,
                                   .arg = NULL};


  thread_descriptor_t sx1276_thd = {.name = "sx1276", .wbase = waSX1276Thread,
                                   .wend = waSX1276Thread + sizeof(waSX1276Thread),
                                   .prio = HIGHPRIO, .funcp = SX1276Thread,
                                   .arg = NULL};

  char charbuf[128 + 2*DTG_RX_BUFFER_SIZE];

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  main_thread = chThdGetSelfX();
  chRegSetThreadName("main");
  
  /* Creating the mailboxes.*/
  chMBObjectInit(&filled_buffers, filled_buffers_queue, DATAGRAM_POOL_SIZE);
  chMBObjectInit(&free_buffers, free_buffers_queue, DATAGRAM_POOL_SIZE);

  /* Pre-filling the free buffers pool with the available buffers. */
  for (int  i = 0; i < DATAGRAM_POOL_SIZE; i++) {
    (void)chMBPostI(&free_buffers, (msg_t)&buffers[i]);
  }

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);
  
  /*
   * Activates SDC driver 1
   */
  //sdcStart(&SDCD1, &sdccfg);
  //InsertHandler();
  //TestSdc();
  //cmd_mount((BaseSequentialStream *)&SD2, 0, NULL);
  //scan_files((BaseSequentialStream *)&SD2, scan_files_path);
  //cmd_umount((BaseSequentialStream *)&SD2, 0, NULL);
  //RemoveHandler();

  /*
   * Creates the radio control thread.
   */
  //sx1272_control_thread = chThdCreate(&sx1272_thd);
  sx1276_control_thread = chThdCreate(&sx1276_thd);
  //rs485_thread = chThdCreateStatic(waRS485Thread, sizeof(waRS485Thread), HIGHPRIO, RS485Thread, NULL);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the user input thread.
   */
#if 1
  shell_thread = chThdCreate(&shell_thd);
#else
  chprintf((BaseSequentialStream *)&SD2, "\r\n\r\n");
  cmd_mount((BaseSequentialStream *)&SD2, 0, NULL);
  cmd_write((BaseSequentialStream *)&SD2, 0, NULL);
  cmd_dir((BaseSequentialStream *)&SD2, 0, NULL);
  cmd_read((BaseSequentialStream *)&SD2, 0, NULL);
  cmd_umount((BaseSequentialStream *)&SD2, 0, NULL);
#endif

  const char *cmd[] = {"on"};
  cmd_radio((BaseSequentialStream *)&SD2, 1, cmd);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    struct dtg_t *dtg;
    msg_t msg = chMBFetchTimeout(&filled_buffers, (msg_t *)&dtg, TIME_INFINITE);

    /* Processing the event.*/
    if (msg == MSG_RESET) {
    }
    else {
      if (dtg) {
        const bool corrected_3outof6_dtg = correct_LorC_field(dtg->payload, dtg->size);
        if (corrected_3outof6_dtg) {
          chMtxLock(&statMtx);
          radioStatistics.LorCfield_corected_dtgs++;
          chMtxUnlock(&statMtx);
        }

        const struct t1_c1_packet_decoder_work_t *t1_c1_decoder = NULL;
        t1_c1_packet_decoder_reset();
        t1_c1_packet_decoder(PACKET_PREAMBLE_DETECTED_MASK, dtg->rssi);

        for (size_t i = 0; i < dtg->size && t1_c1_decoder == NULL; i++)
        {
          for (int j = 8; j--;)  /* Step through bytes, bit for bit. */
          {
            t1_c1_decoder = t1_c1_packet_decoder((dtg->payload[i] >> j) & 1, dtg->rssi);

            if (t1_c1_decoder)
            {
              int charbuflen = 0;
              charbuflen += sprintf(charbuf+charbuflen, "%s;%u;%u;%u;%u;%u;",
                                                        t1_c1_decoder->c1_packet ? "C1": "T1",
                                                        t1_c1_decoder->crc_ok,
                                                        t1_c1_decoder->err_3outof6^1,
                                                        corrected_3outof6_dtg ? 1 : 0,
                                                        dtg->timestamp.millisecond,
                                                        dtg->rssi);

              for (size_t k = 0; k < t1_c1_decoder->L; k++)
                charbuflen += siprintf(charbuf+charbuflen, "%02x", t1_c1_decoder->packet[k]);

              if (stream_dtgs_on)
              {
                chprintf((BaseSequentialStream *)&SD2, "%s\r\n", charbuf);
              }

              break;
            }
          }
        }

        chMtxLock(&statMtx);
        if (t1_c1_decoder)
        {
          if (t1_c1_decoder->crc_ok)
            radioStatistics.crc_good_datagrams++;
          else if (t1_c1_decoder->err_3outof6)
            radioStatistics.err_3out6_datagrams++;
          else
            radioStatistics.crc_error_datagrams++;
        }
        else
        {
          radioStatistics.unrecognized_datagrams++;
        }
        chMtxUnlock(&statMtx);
      }

      (void)chMBPostTimeout(&free_buffers, (msg_t)dtg, TIME_INFINITE);
    }
  }
}
