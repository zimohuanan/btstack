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

#define BTSTACK_FILE__ "broadcast_audio_scan_service_server.c"

#include "ble/att_db.h"
#include "ble/att_server.h"
#include "bluetooth_gatt.h"
#include "btstack_debug.h"
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack_util.h"

#include "ble/gatt-service/broadcast_audio_scan_service_server.h"


static att_service_handler_t    broadcast_audio_scan_service;
static hci_con_handle_t         bass_con_handle;
static btstack_packet_handler_t bass_event_callback;

// characteristic: AUDIO_SCAN_CONTROL_POINT
static uint16_t  bass_audio_scan_control_point_handle;

// characteristic:  RECEIVE_STATE 
static uint16_t  bass_receive_state_handle;
static uint16_t  bass_receive_state_client_configuration_handle;
static uint16_t  bass_receive_state_client_configuration;

static void broadcast_audio_scan_service_server_reset_values(void){
    bass_con_handle = HCI_CON_HANDLE_INVALID;
    bass_receive_state_client_configuration = 0;
}

static void bass_set_con_handle(hci_con_handle_t con_handle, uint16_t configuration){
    bass_con_handle = (configuration == 0) ? HCI_CON_HANDLE_INVALID : con_handle;
}

static uint16_t broadcast_audio_scan_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
    UNUSED(con_handle);

    if (attribute_handle == bass_receive_state_handle){
        // TODO
    }

    if (attribute_handle == bass_receive_state_client_configuration_handle){
        return att_read_callback_handle_little_endian_16(bass_receive_state_client_configuration, offset, buffer, buffer_size);
    }
    return 0;
}

static int broadcast_audio_scan_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    UNUSED(transaction_mode);
    UNUSED(offset);

    if (attribute_handle == bass_audio_scan_control_point_handle){
        // TODO
    }

    else if (attribute_handle == bass_receive_state_client_configuration_handle){
        bass_receive_state_client_configuration = little_endian_read_16(buffer, 0);
        bass_set_con_handle(con_handle, bass_receive_state_client_configuration);
    }
    return 0;
}

static void broadcast_audio_scan_service_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(packet);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET){
        return;
    }

    hci_con_handle_t con_handle;
    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = hci_event_disconnection_complete_get_connection_handle(packet);
            if (bass_con_handle == con_handle){
                broadcast_audio_scan_service_server_reset_values();
            }
            break;
        default:
            break;
    }
}

void broadcast_audio_scan_service_server_init(void){
    // get service handle range
    uint16_t start_handle = 0;
    uint16_t end_handle   = 0xfff;
    int service_found = gatt_server_get_handle_range_for_service_with_uuid16(ORG_BLUETOOTH_SERVICE_BROADCAST_AUDIO_SCAN_SERVICE, &start_handle, &end_handle);
    btstack_assert(service_found != 0);
    UNUSED(service_found);

    broadcast_audio_scan_service_server_reset_values();

    bass_audio_scan_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_AUDIO_SCAN_CONTROL_POINT);

    bass_receive_state_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_RECEIVE_STATE);
    bass_receive_state_client_configuration_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_RECEIVE_STATE);

    log_info("Found BASS service 0x%02x-0x%02x", start_handle, end_handle);

    // register service with ATT Server
    broadcast_audio_scan_service.start_handle   = start_handle;
    broadcast_audio_scan_service.end_handle     = end_handle;
    broadcast_audio_scan_service.read_callback  = &broadcast_audio_scan_service_read_callback;
    broadcast_audio_scan_service.write_callback = &broadcast_audio_scan_service_write_callback;
    broadcast_audio_scan_service.packet_handler = broadcast_audio_scan_service_packet_handler;
    att_server_register_service_handler(&broadcast_audio_scan_service);
}

void broadcast_audio_scan_service_server_register_packet_handler(btstack_packet_handler_t callback){
    btstack_assert(callback != NULL);
    bass_event_callback = callback;
}

