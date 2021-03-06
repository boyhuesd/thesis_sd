/*
 * format.h
 *
 *  Created on: 25-04-2014
 *      Author: boyhuesd
 */

#ifndef FORMAT_H_
#define FORMAT_H_
#include "cirbuf.h"

const uint8_t wavHeader[44] = {
    0x52, 0x49, 0x46, 0x46, // RIFF
    0x00, 0x00, 0x00, 0x00, // Chunk Size
    0x57, 0x41, 0x56, 0x45, // WAVE
    0x66, 0x6d, 0x74, 0x20, // fmt
    0x10, 0x00, 0x00, 0x00, // Subchunk 1 size
    0x01, 0x00, // PCM format
    0x01, 0x00, // Single channel
    0x40, 0x1f, 0x00, 0x00, // Sample rate = 8ksps
    0x80, 0x3e, 0x00, 0x00, // Byte rate = no. of channels * samplerate * bit/sample/8
    0x02, 0x00, // Block align = no. of chan * bit/sam/8
    0x10, 0x00, // Bits per sample
    0x64, 0x61, 0x74, 0x61, // data
    0x00, 0x00, 0x00, 0x00, // Sub chunk2 size (no. of channels * samples * bps/8)
};

#endif /* FORMAT_H_ */
