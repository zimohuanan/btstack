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
 * @title Broadcast Audio Scan Service Server (BASS)
 * 
 */

#ifndef broadcast_audio_scan_service_server_H
#define broadcast_audio_scan_service_server_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define BASS_ERROR_CODE_OPCODE_NOT_SUPPORTED            0x80
#define BASS_ERROR_CODE_INVALID_SOURCE_ID               0x81

/* API_START */

typedef struct {
    btstack_linked_item_t item;

    // characteristic:  RECEIVE_STATE 
    uint16_t bass_receive_state_handle;
    uint16_t bass_receive_state_client_configuration_handle;
    uint16_t bass_receive_state_client_configuration;
} bass_source_t;

/**
 * @text The Broadcast Audio Scan Service is used by servers to expose their status with respect 
 * to synchronization to broadcast Audio Streams and associated data, including Broadcast_Codes 
 * used to decrypt encrypted broadcast Audio Streams. Clients can use the attributes exposed by 
 * servers to observe and/or request changes in server behavior.
 * 
 * To use with your application, add `#import <broadcast_audio_scan_service.gatt>` to your .gatt file. 
 */

/**
 * @brief Init Broadcast Audio Scan Service Server with ATT DB
 */
void broadcast_audio_scan_service_server_init(uint8_t bass_source_num, bass_source_t * sources);

/**
 * @brief Register callback.
 * @param callback
 */
void broadcast_audio_scan_service_server_register_packet_handler(btstack_packet_handler_t callback);

/* API_END */

#if defined __cplusplus
}
#endif

#endif

