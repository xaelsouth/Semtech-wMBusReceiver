/*-
 * Copyright (c) 2019 <xael.south@yandex.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
 

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "t1_c1_packet_decoder.h"
#include "t1_c1_util.h"
 
static const uint8_t TAB_3OUTOF6[16] = {
  0b010110,
  0b001101,
  0b001110,
  0b001011,
  0b011100,
  0b011001,
  0b011010,
  0b010011,
  0b101100,
  0b100101,
  0b100110,
  0b100011,
  0b110100,
  0b110001,
  0b110010,
  0b101001
};

bool correct_LorC_field(uint8_t *dtgpayload, size_t dtgpayloadsize)
{
  if (dtgpayloadsize < t1_3out6_packet_length(12)) return false;

  t1_c1_packet_decoder_reset();
  t1_c1_packet_decoder(PACKET_PREAMBLE_DETECTED_MASK, PACKET_CAPTURE_THRESHOLD);
  for (size_t i = 0; i < 3; i++)
  {
    for (int j = 8; j--;)  /* Step through bytes, bit for bit. */
    {
      t1_c1_packet_decoder((dtgpayload[i] >> j) & 1, PACKET_CAPTURE_THRESHOLD);
    }
  }

  if (t1_c1_packet_decoder_work.err_3outof6)
  {
    uint32_t bitstream = ((uint32_t)dtgpayload[0] << 16) | ((uint32_t)dtgpayload[1] << 8) | (uint32_t)dtgpayload[2];
    //const uint8_t L = HIGH_NIBBLE_3OUTOF6[(bitstream >> 18) & 0b111111] | LOW_NIBBLE_3OUTOF6[(bitstream >> 12) & 0b111111];
    const uint8_t C = HIGH_NIBBLE_3OUTOF6[(bitstream >>  6) & 0b111111] | LOW_NIBBLE_3OUTOF6[(bitstream >>  0) & 0b111111];

    if (C == 0xFF) { /* According to wireless MBus standard, the C field from end device is never 0xFF but 0x44 typically. */
      uint8_t h = (0x44 >> 4) & 0x0F;
      uint8_t l = (0x44 >> 0) & 0x0F;

      bitstream &= 0xFFF000u;
      bitstream |= ((uint32_t)TAB_3OUTOF6[h] << 6);
      bitstream |= ((uint32_t)TAB_3OUTOF6[l] << 0);

      dtgpayload[2] = bitstream >> 0;
    }

    for (unsigned E = 9; E <= 255; ++E)
    {
      uint8_t h = (E >> 4) & 0x0F;
      uint8_t l = (E >> 0) & 0x0F;

      bitstream &= 0x000FFFu;
      bitstream |= ((uint32_t)TAB_3OUTOF6[h] << 18);
      bitstream |= ((uint32_t)TAB_3OUTOF6[l] << 12);

      dtgpayload[0] = bitstream >> 16;
      dtgpayload[1] = bitstream >>  8;

      t1_c1_packet_decoder_reset();
      t1_c1_packet_decoder(PACKET_PREAMBLE_DETECTED_MASK, PACKET_CAPTURE_THRESHOLD);

      /* First data block is 12 byte long, but 12 data bytes + 6 bytes for 3outof6 coding. */
      for (size_t i = 0; i < t1_3out6_packet_length(12); i++)
      {
        for (int j = 8; j--;)  /* Step through bytes, bit for bit. */
        {
          t1_c1_packet_decoder((dtgpayload[i] >> j) & 1, PACKET_CAPTURE_THRESHOLD);
        }
      }

      /* Check CRC only for the first datagram block. */
      if (!t1_c1_packet_decoder_work.err_3outof6) {
        if (check_calc_crc_wmbus(t1_c1_packet_decoder_work.packet, 12))
          return true;
      }
    }
  }

  return false;
}

unsigned cook_pkt(uint8_t *data, unsigned datalen)
{
    uint8_t *dst = data;
    unsigned dstlen = 0;

    if (datalen >= 12)
    {
        dst += 10;
        dstlen += 10;

        data += 12;
        datalen -= 12;

        while (datalen)
        {
            if (datalen >= 18)
            {
                memmove(dst, data, 16);

                dst += 16;
                dstlen += 16;

                data += 18;
                datalen -= 18;
            }
            else
            {
                memmove(dst, data, datalen-2);

                dst += (datalen-2);
                dstlen += (datalen-2);

                data += datalen;
                datalen -= datalen;
            }
        }
    }

    return dstlen;
}

uint8_t t1_c1_get_positive_rssi( int8_t rssi )
{
    if (rssi < -127)
        return 0;
    else if (rssi >= 0)
        return 255;

    return UINT8_C(255 + (int)rssi*2);
}
