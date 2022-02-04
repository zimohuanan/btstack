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
 * @title Published Audio Capabilities Service Server (PACS)
 * 
 */

#ifndef PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER_H
#define PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER_H

#include <stdint.h>
#include "le_audio.h"

#if defined __cplusplus
extern "C" {
#endif

/* API_START */

/**
 * @text The Published Audio Capabilities Service Server exposes server audio capabilities and audio
 * availability. 
 * 
 * To use with your application, add `#import <published_audio_capabilities_service.gatt>` to your .gatt file. 
 */

// Audio capabilities
typedef enum {
    PACS_CODEC_SPECIFIC_CAPABILITY_TYPE_SAMPLING_FREQUENCY = 0x01,
    PACS_CODEC_SPECIFIC_CAPABILITY_TYPE_FRAME_DURATION,
    PACS_CODEC_SPECIFIC_CAPABILITY_TYPE_AUDIO_CHANNEL_ALLOCATION,
    PACS_CODEC_SPECIFIC_CAPABILITY_TYPE_OCTETS_PER_CODEC_FRAME,
    PACS_CODEC_SPECIFIC_CAPABILITY_TYPE_CODEC_FRAME_BLOCKS_PER_SDU,
} pacs_codec_specific_capability_type_t;

typedef enum {
    PACS_CODEC_SAMPLING_FREQUENCY_8000_HZ   = 0x01,
    PACS_CODEC_SAMPLING_FREQUENCY_11025_HZ  = 0x02,
    PACS_CODEC_SAMPLING_FREQUENCY_16000_HZ  = 0x03,
    PACS_CODEC_SAMPLING_FREQUENCY_22050_HZ  = 0x04,
    PACS_CODEC_SAMPLING_FREQUENCY_24000_HZ  = 0x05,
    PACS_CODEC_SAMPLING_FREQUENCY_32000_HZ  = 0x06,
    PACS_CODEC_SAMPLING_FREQUENCY_44100_HZ  = 0x07,
    PACS_CODEC_SAMPLING_FREQUENCY_48000_HZ  = 0x08,
    PACS_CODEC_SAMPLING_FREQUENCY_88200_HZ  = 0x09,
    PACS_CODEC_SAMPLING_FREQUENCY_96000_HZ  = 0x0A,
    PACS_CODEC_SAMPLING_FREQUENCY_176400_HZ = 0x0B,
    PACS_CODEC_SAMPLING_FREQUENCY_192000_HZ = 0x0C,
    PACS_CODEC_SAMPLING_FREQUENCY_384000_HZ = 0x0D
} pacs_codec_sampling_frequency_t;

typedef enum {
    PACS_CODEC_FRAME_DURATION_7_5_MS = 0x00,
    PACS_CODEC_FRAME_DURATION_10_MS = 0x01
} pacs_codec_frame_duration_t;

typedef struct {
    pacs_codec_specific_capability_type_t type;
    const uint8_t * value; // max 5 bytes, value_length is fixed, and depends on type
} pacs_codec_specific_capability_t;

typedef struct {
    le_audio_codec_id_t codec_id; 

    uint8_t codec_specific_capabilities_num;
    const   pacs_codec_specific_capability_t * capabilities;

    uint8_t metadata_length;
    const uint8_t * metadata;
} pacs_record_t;

/**
 * @brief Init Published Audio Capabilities Service Server with ATT DB
 * @param sink_pac_records
 * @param sink_pac_records_num
 * @param source_pac_records
 * @param source_pac_records_num
 */
void published_audio_capabilities_service_server_init(const pacs_record_t * sink_pac_records, uint8_t sink_pac_records_num, const pacs_record_t * source_pac_records, uint8_t source_pac_records_num);

/**
 * @brief Register callback.
 * @param callback
 */
void published_audio_capabilities_service_server_register_packet_handler(btstack_packet_handler_t callback);

/* API_END */

#if defined __cplusplus
}
#endif

#endif

