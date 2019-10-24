#ifndef T1_C1_UTIL_H
#define T1_C1_UTIL_H

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
 
/* The idea behind that is to set L and C fields (mode T1 datagrams) to a correct value
 * if one of them is not properly 3outof6 encoded. We will try to change first 24 bits
 * (whereas the L and C fields are encoded) and check CRC of the data in the first block.
 *
 * Measurements shown that for about a fifth datagrams L and/or C fields could be corrected,
 * which does not mean that the full datagram could be saved from being dropped!
 *
 * The next is not implemented:
 *
 * 1) Amount of iterations from a maximum of 255-9+1 could be reduced by observing Hamming
 * distance included in the channel coding itself.
 *
 * 2) In that way the full datagram could be corrected.
 *
 */
bool correct_LorC_field(uint8_t *dtgpayload, size_t dtgpayloadsize);

/** @brief Strip CRCs in place. */
unsigned cook_pkt(uint8_t *data, unsigned datalen);

uint8_t t1_c1_get_positive_rssi( int8_t rssi );

#endif
