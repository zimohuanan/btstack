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

typedef enum {
   BASS_OPCODE_REMOTE_SCAN_STOPPED = 0x00,
   BASS_OPCODE_REMOTE_SCAN_STARTED,
   BASS_OPCODE_ADD_SOURCE,
   BASS_OPCODE_MODIFY_SOURCE,
   BASS_OPCODE_SET_BROADCAST_CODE,
   BASS_OPCODE_REMOVE_SOURCE, 
   BASS_OPCODE_RFU
} bass_opcode_t;

static att_service_handler_t    broadcast_audio_scan_service;
static hci_con_handle_t         bass_con_handle;
static btstack_packet_handler_t bass_event_callback;

// characteristic: AUDIO_SCAN_CONTROL_POINT
static uint16_t bass_audio_scan_control_point_handle;

static btstack_linked_list_t bass_sources;
static uint8_t  bass_sources_num = 0;
static uint8_t  bass_source_counter = 0;
static uint8_t  bass_update_counter = 0;

static uint8_t bass_get_next_source_id(void){
    uint8_t next_source_id;
    if (bass_source_counter == 0xff) {
        next_source_id = 1;
    } else {
        next_source_id = bass_source_counter + 1;
    }
    return next_source_id;
}

static uint8_t bass_get_next_update_counter(void){
    uint8_t next_update_counter;
    if (bass_update_counter == 0xff) {
        next_update_counter = 0;
    } else {
        next_update_counter = bass_update_counter + 1;
    }
    return next_update_counter;
}
static bass_source_t * bass_find_receive_state_for_value_handle(uint16_t attribute_handle){
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, &bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        if (attribute_handle != item->bass_receive_state_handle) continue;
        return item;
    }
    return NULL;
}

static bass_source_t * bass_find_receive_state_for_client_configuration_handle(uint16_t attribute_handle){
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, &bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        if (attribute_handle != item->bass_receive_state_handle) continue;
        return item;
    }
    return NULL;
}

static void broadcast_audio_scan_service_server_reset_values(void){
    bass_con_handle = HCI_CON_HANDLE_INVALID;
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, &bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        item->bass_receive_state_client_configuration = 0;
    }
}

static void bass_set_con_handle(hci_con_handle_t con_handle, uint16_t configuration){
    bass_con_handle = (configuration == 0) ? HCI_CON_HANDLE_INVALID : con_handle;
}

static void bass_emit_remote_scan_stoped(void){
    btstack_assert(bass_event_callback != NULL);
    
    uint8_t event[5];
    uint8_t pos = 0;
    event[pos++] = HCI_EVENT_GATTSERVICE_META;
    event[pos++] = sizeof(event) - 2;
    event[pos++] = GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STOPED;
    little_endian_store_16(event, pos, bass_con_handle);
    pos += 2;
    (*bass_event_callback)(HCI_EVENT_PACKET, 0, event, sizeof(event));
}

static void bass_emit_remote_scan_started(void){
    btstack_assert(bass_event_callback != NULL);
    
    uint8_t event[5];
    uint8_t pos = 0;
    event[pos++] = HCI_EVENT_GATTSERVICE_META;
    event[pos++] = sizeof(event) - 2;
    event[pos++] = GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STARTED;
    little_endian_store_16(event, pos, bass_con_handle);
    pos += 2;
    (*bass_event_callback)(HCI_EVENT_PACKET, 0, event, sizeof(event));
}


// help with buffer == NULL
static uint16_t bass_virtual_memcpy(
    const uint8_t * field_data, uint16_t field_len, uint16_t field_offset,
    uint8_t * buffer, uint16_t buffer_size, uint16_t buffer_offset){

    // only calc total size
    if (buffer == NULL) {
        return field_len;
    }
    return btstack_virtual_memcpy(field_data, field_len, field_offset, buffer, buffer_size, buffer_offset);
}

// offset gives position into fully serialized pacs record
static uint16_t bass_store_source(bass_source_t * source, uint16_t read_offset, uint8_t * buffer, uint16_t buffer_size){
    uint8_t  field_data[16];
    uint16_t source_offset = 0;
    uint16_t stored_bytes = 0;
    memset(buffer, 0, buffer_size);

    field_data[0] = source->source_id;
    stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
    source_offset++;

    field_data[0] = (uint8_t)source->address_type;
    stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
    source_offset++;
    
    reverse_bd_addr(source->address, &field_data[0]);
    stored_bytes += bass_virtual_memcpy(field_data, 6, source_offset, buffer, buffer_size, read_offset);
    source_offset += 6;

    field_data[0] = source->adv_sid;
    stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
    source_offset++;

    little_endian_store_24(field_data, 0, source->broadcast_id);
    stored_bytes += bass_virtual_memcpy(field_data, 3, source_offset, buffer, buffer_size, read_offset);
    source_offset += 3;

    field_data[0] = (uint8_t)source->pa_sync_state;
    stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
    source_offset++;

    field_data[0] = (uint8_t)source->big_encryption;
    stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
    source_offset++;

    if (source->big_encryption == LEA_BIG_ENCRYPTION_BAD_CODE){
        reverse_128(source->bad_code, &field_data[0]);
        stored_bytes += bass_virtual_memcpy(field_data, 16, source_offset, buffer, buffer_size, read_offset);
        source_offset += 16;
    }

    field_data[0] = (uint8_t)source->subgroups_num;
    stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
    source_offset++;

    if (source->subgroups_num == 0){
        return stored_bytes;
    }

    uint8_t i;
    for (i = 0; i < source->subgroups_num; i++){
        bass_subgroup_t subgroup = source->subgroups[source->subgroups_num];

        little_endian_store_32(field_data, 0, subgroup.bis_sync_state);
        stored_bytes += bass_virtual_memcpy(field_data, 4, source_offset, buffer, buffer_size, read_offset);
        source_offset += 4;

        field_data[0] = subgroup.metadata_length;
        stored_bytes += bass_virtual_memcpy(field_data, 1, source_offset, buffer, buffer_size, read_offset);
        source_offset++;

        stored_bytes += bass_virtual_memcpy(subgroup.metadata, subgroup.metadata_length, source_offset, buffer, buffer_size, read_offset);
        source_offset += subgroup.metadata_length;
    }
    
    return stored_bytes;
}

static uint16_t broadcast_audio_scan_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
    UNUSED(con_handle);

    bass_source_t * source;

    source = bass_find_receive_state_for_value_handle(attribute_handle);
    if (source){
        return bass_store_source(source, offset, buffer, buffer_size);
    }

    source = bass_find_receive_state_for_client_configuration_handle(attribute_handle);
    if (source){
        return att_read_callback_handle_little_endian_16(source->bass_receive_state_client_configuration, offset, buffer, buffer_size);
    }

    return 0;
}

static int broadcast_audio_scan_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    UNUSED(transaction_mode);
    UNUSED(offset);
    
    if (attribute_handle == bass_audio_scan_control_point_handle){
        if (buffer_size != 1){
            return BASS_ERROR_CODE_OPCODE_NOT_SUPPORTED;
        }

        bass_opcode_t opcode = (bass_opcode_t)buffer[0];
        switch (opcode){
            case BASS_OPCODE_REMOTE_SCAN_STOPPED:
                bass_emit_remote_scan_stoped();
                break;

            case BASS_OPCODE_REMOTE_SCAN_STARTED:
                bass_emit_remote_scan_started();
                break;

            case BASS_OPCODE_ADD_SOURCE:
                break;

            case BASS_OPCODE_MODIFY_SOURCE:
                break;

            case BASS_OPCODE_SET_BROADCAST_CODE:
                break;

            case BASS_OPCODE_REMOVE_SOURCE:
                break;

            default:
                return BASS_ERROR_CODE_OPCODE_NOT_SUPPORTED;
        }   
    }

    else {
        bass_source_t * source = bass_find_receive_state_for_client_configuration_handle(attribute_handle);
        if (source){
            source->bass_receive_state_client_configuration = little_endian_read_16(buffer, 0);
            bass_set_con_handle(con_handle, source->bass_receive_state_client_configuration);
        }
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

void broadcast_audio_scan_service_server_init(uint8_t sources_num, bass_source_t * sources){
    // get service handle range
    uint16_t start_handle = 0;
    uint16_t end_handle   = 0xfff;
    int service_found = gatt_server_get_handle_range_for_service_with_uuid16(ORG_BLUETOOTH_SERVICE_BROADCAST_AUDIO_SCAN_SERVICE, &start_handle, &end_handle);
    btstack_assert(service_found != 0);
    UNUSED(service_found);

    broadcast_audio_scan_service_server_reset_values();

    bass_audio_scan_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_AUDIO_SCAN_CONTROL_POINT);

    while ( (start_handle < end_handle) && (bass_sources_num < sources_num)) {
        uint16_t chr_value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_RECEIVE_STATE);
        uint16_t chr_client_configuration_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_RECEIVE_STATE);
        log_info("BASS Receive State: value handle 0x%02x, CCC 0x%02x", chr_value_handle, chr_client_configuration_handle);
        
        if (chr_value_handle == 0){
            break;
        }
        bass_source_t * source = &sources[bass_sources_num];
        source->source_id = bass_get_next_source_id();
        source->update_counter = bass_get_next_update_counter();
        
        btstack_linked_list_add(&bass_sources, (btstack_linked_item_t *)source);
        start_handle = chr_client_configuration_handle + 1;
        bass_sources_num++;
    }

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

