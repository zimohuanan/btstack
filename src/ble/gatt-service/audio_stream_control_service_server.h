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
 * @title Audio Stream Sontrol Service Server (ASCS)
 * 
 */

#ifndef AUDIO_STREAM_CONTROL_SERVICE_SERVER_H
#define AUDIO_STREAM_CONTROL_SERVICE_SERVER_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/* API_START */

/**
 * @text The Audio Stream Sontrol Service Server exposes an interface fo Audio Stream Endpoints (ASEs). This enables clients
 * to discover, configure, establish and control the ASEs and their unicast Audio Streams.
 * 
 * To use with your application, add `#import <audio_stream_control_service.gatt>` to your .gatt file. 
 */

#define ASCS_STREAMENDPOINTS_MAX_NUM 5
#define ASCS_CLIENTS_MAX_NUM 5
#define ASCS_STREAMENDPOINT_STATE_MAX_SIZE 5

typedef enum {
    ASCS_ROLE_SINK = 0,
    ASCS_ROLE_SOURCE
} ascs_role_t;

typedef enum {
    ASCS_STATE_IDLE = 0x00,
    ASCS_STATE_CODEC_CONFIGURED,
    ASCS_STATE_QOS_CONFIGURED,
    ASCS_STATE_ENABLING,
    ASCS_STATE_STREAMING,
    ASCS_STATE_DISABLING,
    ASCS_STATE_RELEASING,
    ASCS_STATE_RFU,
} ascs_state_t;

// The ASCS Server must expose at least one Audio Stream Endpoint (ASE) for every Audio Stream it is capable of supporting.
// For each ASE, the ASCS Server maintains an instance of a state machine for each connected client. The state of an ASE is
// controlled by each client, although in some cases the Server can autonomously change the state of an ASE.

typedef struct {
    uint8_t  ase_id;
    ascs_role_t role;

    // prefered server values
    
    uint16_t value_handle;
    uint16_t client_configuration_handle;
    uint16_t client_configuration;
} ascs_streamendpoint_characteristic_t;

typedef struct {
    ascs_streamendpoint_characteristic_t * ase_characteristic;
    ascs_state_t state;
} ascs_streamendpoint_t;

typedef struct {
    hci_con_handle_t con_handle;
    ascs_streamendpoint_t streamendpoints[ASCS_STREAMENDPOINTS_MAX_NUM];
} ascs_remote_client_t;


/**
 * @brief Init Audio Stream Sontrol Service Server with ATT DB
 */
void audio_stream_control_service_server_init(
    const uint8_t streamendpoint_characteristics_num, ascs_streamendpoint_characteristic_t * streamendpoint_characteristics, 
    const uint8_t clients_num, ascs_remote_client_t * clients);

/**
 * @brief Register callback.
 * @param callback
 */
void audio_stream_control_service_server_register_packet_handler(btstack_packet_handler_t callback);

// void audio_stream_control_service_server_set_ase_state(void);

void audio_stream_control_service_server_deinit(void);

/* API_END */

#if defined __cplusplus
}
#endif

#endif

