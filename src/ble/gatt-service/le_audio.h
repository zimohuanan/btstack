/*
 * Copyright (C) 2022 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

/**
 * @title Volume Offset Control Service Server
 * 
 */

#ifndef LE_AUDIO_H
#define LE_AUDIO_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

// Generic Audio/Audio Location Definitions/Bitmap
#define LE_AUDIO_LOCATION_NOT_ALLOWED             0x00000000
#define LE_AUDIO_LOCATION_FRONT_LEFT              0x00000001
#define LE_AUDIO_LOCATION_FRONT_RIGHT             0x00000002
#define LE_AUDIO_LOCATION_FRONT_CENTER            0x00000004
#define LE_AUDIO_LOCATION_LOW_FREQUENCY_EFFECTS1  0x00000008
#define LE_AUDIO_LOCATION_BACK_LEFT               0x00000010
#define LE_AUDIO_LOCATION_BACK_RIGHT              0x00000020
#define LE_AUDIO_LOCATION_FRONT_LEFT_OF_CENTER    0x00000040
#define LE_AUDIO_LOCATION_FRONT_RIGHT_OF_CENTER   0x00000080
#define LE_AUDIO_LOCATION_BACK_CENTER             0x00000100
#define LE_AUDIO_LOCATION_LOW_FREQUENCY_EFFECTS2  0x00000200
#define LE_AUDIO_LOCATION_SIDE_LEFT               0x00000400
#define LE_AUDIO_LOCATION_SIDE_RIGHT              0x00000800
#define LE_AUDIO_LOCATION_TOP_FRONT_LEFT          0x00001000
#define LE_AUDIO_LOCATION_TOP_FRONT_RIGHT         0x00002000
#define LE_AUDIO_LOCATION_TOP_FRONT_CENTER        0x00004000
#define LE_AUDIO_LOCATION_TOP_CENTER              0x00008000
#define LE_AUDIO_LOCATION_TOP_BACK_LEFT           0x00010000
#define LE_AUDIO_LOCATION_TOP_BACK_RIGHT          0x00020000
#define LE_AUDIO_LOCATION_TOP_SIDE_LEFT           0x00040000
#define LE_AUDIO_LOCATION_TOP_SIDE_RIGHT          0x00080000
#define LE_AUDIO_LOCATION_TOP_BACK_CENTER         0x00100000
#define LE_AUDIO_LOCATION_BOTTOM_FRONT_CENTER     0x00200000
#define LE_AUDIO_LOCATION_BOTTOM_FRONT_LEFT       0x00400000
#define LE_AUDIO_LOCATION_BOTTOM_FRONT_RIGHT      0x00800000
#define LE_AUDIO_LOCATION_FRONT_LEFT_WIDE         0x01000000
#define LE_AUDIO_LOCATION_FRONT_RIGHT_WIDE        0x02000000
#define LE_AUDIO_LOCATION_LEFT_SURROUND           0x04000000
#define LE_AUDIO_LOCATION_RIGHT_SURROUND          0x08000000
#define LE_AUDIO_LOCATION_RFU                     0x10000000

typedef enum {
    LEA_CODEC_SAMPLING_FREQUENCY_8000_HZ   = 0x01,
    LEA_CODEC_SAMPLING_FREQUENCY_11025_HZ  = 0x02,
    LEA_CODEC_SAMPLING_FREQUENCY_16000_HZ  = 0x03,
    LEA_CODEC_SAMPLING_FREQUENCY_22050_HZ  = 0x04,
    LEA_CODEC_SAMPLING_FREQUENCY_24000_HZ  = 0x05,
    LEA_CODEC_SAMPLING_FREQUENCY_32000_HZ  = 0x06,
    LEA_CODEC_SAMPLING_FREQUENCY_44100_HZ  = 0x07,
    LEA_CODEC_SAMPLING_FREQUENCY_48000_HZ  = 0x08,
    LEA_CODEC_SAMPLING_FREQUENCY_88200_HZ  = 0x09,
    LEA_CODEC_SAMPLING_FREQUENCY_96000_HZ  = 0x0A,
    LEA_CODEC_SAMPLING_FREQUENCY_176400_HZ = 0x0B,
    LEA_CODEC_SAMPLING_FREQUENCY_192000_HZ = 0x0C,
    LEA_CODEC_SAMPLING_FREQUENCY_384000_HZ = 0x0D
} lea_codec_sampling_frequency_t;

typedef enum {
    LE_AUDIO_CODING_FORMAT_LC3 = 0x06
} lea_coding_format_t;

// struct for codec id
typedef struct {
    lea_coding_format_t coding_format;
    uint16_t company_id;
    uint16_t vendor_specific_codec_id;
} lea_codec_id_t;

#if defined __cplusplus
}
#endif

#endif

