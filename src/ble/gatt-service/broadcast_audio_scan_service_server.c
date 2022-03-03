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

#include <stdio.h>

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
    bass_source_counter = next_source_id;
    return next_source_id;
}

static uint8_t bass_get_next_update_counter(void){
    uint8_t next_update_counter;
    if (bass_update_counter == 0xff) {
        next_update_counter = 0;
    } else {
        next_update_counter = bass_update_counter + 1;
    }
    bass_update_counter = next_update_counter;
    return next_update_counter;
}

// returns positive number if counter a > b
static int8_t bass_counter_delta(uint8_t counter_a, uint8_t counter_b){
    return (int8_t)(counter_a - counter_b);
}

static bass_source_t * bass_find_empty_or_least_used_source(void){
    bass_source_t * least_used_source = NULL;

    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, &bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        if (item->source_id == 0){
            return item;
        }
        if (!least_used_source){
            least_used_source = item;
            continue;
        }
        if (bass_counter_delta(item->update_counter, least_used_source->update_counter) < 0 ){
            least_used_source = item;
        }
    }
    btstack_assert(least_used_source != NULL);
    return least_used_source;
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
        if (attribute_handle != item->bass_receive_state_client_configuration_handle) continue;
        return item;
    }
    return NULL;
}

static bass_source_t * bass_find_source_for_source_id(uint8_t source_id){
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, &bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        if (source_id != item->source_id) continue;
        return item;
    }
    return NULL;
}

static void bass_server_reset_values(void){
    bass_con_handle = HCI_CON_HANDLE_INVALID;
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, &bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        memset(item, 0, sizeof(bass_source_t));
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

static void bass_emit_remote_pa_sync_state(bass_source_t * source, uint8_t * buffer){
    btstack_assert(bass_event_callback != NULL);
    
    uint8_t event[20];
    uint8_t pos = 0;
    event[pos++] = HCI_EVENT_GATTSERVICE_META;
    event[pos++] = sizeof(event) - 2;
    event[pos++] = GATTSERVICE_SUBEVENT_BASS_PA_SYNC_STATE;
    little_endian_store_16(event, pos, bass_con_handle);
    pos += 2;
    
    event[pos++] = source->source_id;
    event[pos++] = (uint8_t)source->address_type;
    memcpy(&event[pos], source->address, 6);
    event[pos++] = source->adv_sid;
    little_endian_store_24(event, pos, source->broadcast_id);
    pos += 3;
    event[pos++] = source->pa_sync_state;
    little_endian_store_16(event, pos, source->pa_interval);
    pos += 2;
    
    (*bass_event_callback)(HCI_EVENT_PACKET, 0, event, sizeof(event));
}

static void bass_emit_broadcast_code(uint8_t * buffer){
    btstack_assert(bass_event_callback != NULL);
    
    uint8_t event[22];
    uint8_t pos = 0;
    event[pos++] = HCI_EVENT_GATTSERVICE_META;
    event[pos++] = sizeof(event) - 2;
    event[pos++] = GATTSERVICE_SUBEVENT_BASS_BROADCAST_CODE;
    little_endian_store_16(event, pos, bass_con_handle);
    pos += 2;
    event[pos++] = buffer[0];
    reverse_bytes(&buffer[1], &event[pos], 16);
    pos += 16;

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
static uint16_t bass_copy_source(bass_source_t * source, uint16_t read_offset, uint8_t * buffer, uint16_t buffer_size){
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
        return bass_copy_source(source, offset, buffer, buffer_size);
    }

    source = bass_find_receive_state_for_client_configuration_handle(attribute_handle);
    if (source){
        return att_read_callback_handle_little_endian_16(source->bass_receive_state_client_configuration, offset, buffer, buffer_size);
    }

    return 0;
}

static bool bass_pa_info_and_subgroups_valid(uint8_t *buffer, uint16_t buffer_size){
    uint8_t pos = 0;
    // pa_sync_state
    uint8_t pa_sync = buffer[pos++];
    if (pa_sync >= (uint8_t)LEA_PA_SYNC_RFU){
        log_info("Unexpected pa_sync 0x%02X", pa_sync);
        return false;
    }

    // pa_interval
    pos += 2;
    uint8_t num_subgroups = buffer[pos++];
    if (num_subgroups > BASS_SUBGROUPS_MAX_NUM){
        log_info("Number of subgroups %u exceedes maximum %u", num_subgroups, BASS_SUBGROUPS_MAX_NUM);
        return false;
    }

    // printf("Number of subgroups %u, buffer size %u, pos %u\n", num_subgroups, buffer_size, pos);
    uint8_t i;
    for (i = 0; i < num_subgroups; i++){
        // bis_sync
        pos += 4;
        
        // check if we can read metadata_length
        if (pos >= buffer_size){
            log_info("Metadata length not specified, subgroup %u", i);
            return false;
        }
        
        uint8_t metadata_length = buffer[pos++];
        if (metadata_length > BASS_METADATA_MAX_LENGTH){
            log_info("Metadata length %u exceedes maximum %u", metadata_length, BASS_METADATA_MAX_LENGTH);
            return false;
        }    
        // metadata
        pos += metadata_length;
    }
    return (pos == buffer_size);
}

static bool bass_remote_add_source_buffer_valid(uint8_t *buffer, uint16_t buffer_size){
    if (buffer_size < 15){ 
        log_info("Add Source opcode, buffer too small");
        return false;
    }

    uint8_t pos = 0;
    // addr type
    uint8_t adv_type = buffer[pos++];
    if (adv_type > (uint8_t)BD_ADDR_TYPE_LE_RANDOM){
        log_info("Unexpected adv_type 0x%02X", adv_type);
        return false;
    }

    // address
    pos += 6;

    // advertising_sid Range: 0x00-0x0F
    uint8_t advertising_sid = buffer[pos++];
    if (advertising_sid > 0x0F){
        log_info("Advertising sid out of range 0x%02X", advertising_sid);
        return false;
    }

    // broadcast_id
    pos += 3;
    return bass_pa_info_and_subgroups_valid(buffer+pos, buffer_size-pos);
}

static bool bass_remote_modify_source_buffer_valid(uint8_t *buffer, uint16_t buffer_size){
    if (buffer_size < 10){ 
        log_info("Modify Source opcode, buffer too small");
        return false;
    }
    
    uint8_t pos = 1; // source_id
    return bass_pa_info_and_subgroups_valid(buffer+pos, buffer_size-pos);
}

static void broadcast_audio_scan_service_server_sync_info_request(bass_source_t * source){
    // TODO notify client
    // TODO start timer
    
}

static void bass_add_pa_info_and_subgroups(bass_source_t * source, uint8_t *buffer, uint16_t buffer_size){
    uint8_t pos = 0;
    lea_pa_sync_t pa_sync = (lea_pa_sync_t)buffer[pos++]; 
    switch (pa_sync){
        case LEA_PA_SYNC_DO_NOT_SYNCHRONIZE_TO_PA:
            source->pa_sync_state = LEA_PA_SYNC_STATE_NOT_SYNCHRONIZED_TO_PA;
            break;
        case LEA_PA_SYNC_SYNCHRONIZE_TO_PA_PAST_AVAILABLE:
        case LEA_PA_SYNC_SYNCHRONIZE_TO_PA_PAST_NOT_AVAILABLE:
            source->pa_sync_state = LEA_PA_SYNC_STATE_SYNCINFO_REQUEST;
            break;
        default:
            btstack_assert(false);
            return;
    }
    bass_emit_remote_pa_sync_state(source, buffer);
    
    source->pa_interval = little_endian_read_16(buffer, pos);
    pos += 2;

    source->subgroups_num = buffer[pos++];

    uint8_t i;
    for (i = 0; i < source->subgroups_num; i++){
        // bis_sync
        source->subgroups[i].bis_sync_state = little_endian_read_32(buffer, pos);
        pos += 4;
        source->subgroups[i].metadata_length = buffer[pos++];
        // metadata
        memcpy(&source->subgroups[i].metadata[0], &buffer[pos], source->subgroups[i].metadata_length);
        pos += source->subgroups[i].metadata_length;
    }
}

static void bass_add_source(uint8_t *buffer, uint16_t buffer_size){
    UNUSED(buffer_size);
    bass_source_t * source = bass_find_empty_or_least_used_source();
    if (source == NULL){
        return;
    }

    uint8_t pos = 0;
    source->address_type = (bd_addr_type_t)buffer[pos++];
    
    reverse_bd_addr(&buffer[pos], source->address);
    pos += 6;

    source->adv_sid = buffer[pos++];

    source->broadcast_id = little_endian_read_24(buffer, pos);
    pos += 3;

    bass_add_pa_info_and_subgroups(source, buffer+pos, buffer_size-pos);
}

static void bass_modify_source(uint8_t *buffer, uint16_t buffer_size){
    UNUSED(buffer_size);
    uint8_t pos = 0;

    uint8_t source_id = buffer[pos++];

    bass_source_t * source = bass_find_source_for_source_id(source_id);
    if (source == NULL){
        return;
    }
    bass_add_pa_info_and_subgroups(source, buffer+pos, buffer_size-pos);
}

static int broadcast_audio_scan_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    UNUSED(transaction_mode);
    UNUSED(offset);
    
    if (attribute_handle == bass_audio_scan_control_point_handle){
        if (buffer_size < 1){
            return BASS_ERROR_CODE_OPCODE_NOT_SUPPORTED;
        }

        lea_bass_opcode_t opcode = (lea_bass_opcode_t)buffer[0];
        uint8_t  *remote_data = &buffer[1];
        uint16_t remote_data_size = buffer_size - 1;

        switch (opcode){
            case LEA_BASS_OPCODE_REMOTE_SCAN_STOPPED:
                bass_emit_remote_scan_stoped();
                break;

            case LEA_BASS_OPCODE_REMOTE_SCAN_STARTED:
                bass_emit_remote_scan_started();
                break;

            case LEA_BASS_OPCODE_ADD_SOURCE:
                if (bass_remote_add_source_buffer_valid(remote_data, remote_data_size)){
                    bass_add_source(remote_data, remote_data_size);
                }
                break;

            case LEA_BASS_OPCODE_MODIFY_SOURCE:
                if (bass_remote_modify_source_buffer_valid(remote_data, remote_data_size)){
                    bass_modify_source(remote_data, remote_data_size);
                }
                break;

            case LEA_BASS_OPCODE_SET_BROADCAST_CODE:
                if (remote_data_size < 17){
                    break;
                }
                bass_emit_broadcast_code(remote_data);
                break;

            case LEA_BASS_OPCODE_REMOVE_SOURCE:
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
            bass_sources_num = 0;
            if (bass_con_handle == con_handle){
                bass_server_reset_values();
            }
            break;
        default:
            break;
    }
}

void broadcast_audio_scan_service_server_init(uint8_t sources_num, bass_source_t * sources){
    // get service handle range
    uint16_t start_handle = 0;
    uint16_t end_handle   = 0xffff;
    int service_found = gatt_server_get_handle_range_for_service_with_uuid16(ORG_BLUETOOTH_SERVICE_BROADCAST_AUDIO_SCAN_SERVICE, &start_handle, &end_handle);
    btstack_assert(service_found != 0);
    UNUSED(service_found);

    bass_audio_scan_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_AUDIO_SCAN_CONTROL_POINT);
    bass_sources_num = 0;
    bass_source_counter = 0;
    bass_update_counter = 0;

    while ( (start_handle < end_handle) && (bass_sources_num < sources_num)) {
        uint16_t chr_value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_RECEIVE_STATE);
        uint16_t chr_client_configuration_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_RECEIVE_STATE);
        log_info("BASS Receive State: value handle 0x%02x, CCC 0x%02x", chr_value_handle, chr_client_configuration_handle);
        
        if (chr_value_handle == 0){
            break;
        }
        bass_source_t * source = &sources[bass_sources_num];
        memset(source, 0, sizeof(bass_source_t));

        source->source_id = bass_get_next_source_id();
        source->update_counter = bass_get_next_update_counter();
        
        source->bass_receive_state_handle = chr_value_handle;
        source->bass_receive_state_client_configuration_handle = chr_client_configuration_handle;
        source->bass_receive_state_client_configuration = 0;

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


void broadcast_audio_scan_service_server_set_pa_sync_state(uint8_t source_id, lea_pa_sync_state_t sync_state){
    btstack_assert(sync_state >= LEA_PA_SYNC_STATE_RFU);

    bass_source_t * source = bass_find_source_for_source_id(source_id);
    if (!source){
        return;
    }

    source->pa_sync_state = sync_state;

    switch (sync_state){
        case LEA_PA_SYNC_STATE_SYNCINFO_REQUEST:
            broadcast_audio_scan_service_server_sync_info_request(source);
            break;
        default:
            break;
    }   
}

btstack_linked_list_t * broadcast_audio_scan_service_server_get_sources(void){
    return &bass_sources;
}
