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

#define BTSTACK_FILE__ "audio_stream_control_service_server.c"

#include "ble/att_db.h"
#include "ble/att_server.h"
#include "bluetooth_gatt.h"
#include "btstack_debug.h"
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack_util.h"

#include "ble/gatt-service/audio_stream_control_service_server.h"

#define ASCS_TASK_SEND_VALUE_CHANGED                                 0x01
#define ASCS_TASK_SEND_CODEC_CONFIGURED_CONTROL_POINT_RESPONSE       0x02

static att_service_handler_t    audio_stream_control_service;
static btstack_packet_handler_t ascs_event_callback;

static ascs_streamendpoint_characteristic_t * ascs_streamendpoint_characteristics;
static uint8_t  ascs_streamendpoint_chr_num = 0;
static ascs_remote_client_t * ascs_clients;
static uint8_t ascs_clients_num = 0;
static uint8_t ascs_streamendpoint_characteristics_id_counter = 0;

// characteristic: ASE_CONTROL_POINT
static uint16_t ascs_ase_control_point_handle;
static uint16_t ascs_ase_control_point_client_configuration_handle;
static uint16_t ascs_ase_control_point_client_configuration_value;

static uint8_t ascs_get_next_streamendpoint_chr_id(void){
    uint8_t next_streamendpoint_id;
    if (ascs_streamendpoint_characteristics_id_counter == 0xff) {
        next_streamendpoint_id = 1;
    } else {
        next_streamendpoint_id = ascs_streamendpoint_characteristics_id_counter + 1;
    }
    ascs_streamendpoint_characteristics_id_counter = next_streamendpoint_id;
    return next_streamendpoint_id;
}

static ascs_remote_client_t * ascs_get_remote_client_for_con_handle(hci_con_handle_t con_handle){
    uint8_t i;
    for (i = 0; i < ascs_clients_num; i++){
        if (ascs_clients[i].con_handle == con_handle){
            return &ascs_clients[i];
        }
    }
    return NULL;
}

static ascs_streamendpoint_t * ascs_get_streamendpoint_for_ase_id(ascs_remote_client_t * client, uint8_t ase_id){
    uint8_t i;
    for (i = 0; i < ASCS_STREAMENDPOINTS_MAX_NUM; i++){
        if (client->streamendpoints[i].ase_characteristic->ase_id == ase_id){
            return &client->streamendpoints[i];
        }
    }
    return NULL;
}

static uint16_t audio_stream_control_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
    UNUSED(con_handle);
    
    // if (attribute_handle == ascs_sink_ase_handle){
    //     // TODO
    // }

    // if (attribute_handle == ascs_source_ase_handle){
    //     // TODO
    // }

    // if (attribute_handle == ascs_sink_ase_client_configuration_handle){
    //     return att_read_callback_handle_little_endian_16(ascs_sink_ase_client_configuration, offset, buffer, buffer_size);
    // }
    
    // if (attribute_handle == ascs_source_ase_client_configuration_handle){
    //     return att_read_callback_handle_little_endian_16(ascs_source_ase_client_configuration, offset, buffer, buffer_size);
    // }

    return 0;
}

static uint16_t ascs_get_client_codec_config_operation_length(uint8_t ase_num, uint8_t *buffer, uint16_t buffer_size){
    uint8_t i = 0;
    uint16_t pos = 0;
    uint8_t codec_config_len;

    for (i = 0; i < ase_num; i++){
        if ( (buffer_size - pos) < 9 ){
            return 0;
        }
        pos += 8; // ase_id, latency, phy, codec_id, codec config len
        codec_config_len = buffer[pos++];
        if ( (buffer_size - pos) < codec_config_len ){
            return 0;
        }
        pos += codec_config_len; 
    }
    return pos;
}

static void ascs_update_response_code(ascs_streamendpoint_t * streamendpoint, uint8_t new_response_code, uint8_t new_reason, uint8_t * resulting_response_code){
    // return via notification only the first error for streamendpoint
    if (streamendpoint->control_point_response_code == ASCS_ERROR_CODE_SUCCESS){
        streamendpoint->control_point_response_code = new_response_code;
        streamendpoint->control_point_reason = new_reason;
    }

    // as write result return the first error that occured
    if (*resulting_response_code == ASCS_ERROR_CODE_SUCCESS){
        *resulting_response_code = new_response_code;
    }
}


static void audio_stream_control_service_server_can_send_now(void * context){
    ascs_remote_client_t * client = (ascs_remote_client_t *) context;

    if ((client->scheduled_tasks & ASCS_TASK_SEND_CODEC_CONFIGURED_CONTROL_POINT_RESPONSE) != 0){
        client->scheduled_tasks &= ~ASCS_TASK_SEND_CODEC_CONFIGURED_CONTROL_POINT_RESPONSE;

        uint8_t value[3];
        uint8_t i;
        uint8_t ase_num = 0;
        for (i = 0; i < ASCS_STREAMENDPOINTS_MAX_NUM; i++){
            ascs_streamendpoint_t streamendpoint = client->streamendpoints[i];
            if (streamendpoint.control_point_addressed){
                streamendpoint.control_point_addressed = false;
                streamendpoint.w4_server_confirmation = true;

                // TODO
                ase_num++;
            }
        }
        if (ase_num > 0){
            att_server_notify(client->con_handle, ascs_ase_control_point_client_configuration_handle, &value[0], sizeof(value));
        }

    } else if ((client->scheduled_tasks & ASCS_TASK_SEND_VALUE_CHANGED) != 0){
        bool notification_sent = false;

        uint8_t i;
        for (i = 0; i < ASCS_STREAMENDPOINTS_MAX_NUM; i++){
            ascs_streamendpoint_t streamendpoint = client->streamendpoints[i];
            if (!streamendpoint.value_changed){
                continue;
            }

            if (!notification_sent){
                streamendpoint.value_changed = false;
                notification_sent = true;
                uint8_t value[3];
                // TODO
                att_server_notify(client->con_handle, streamendpoint.ase_characteristic->value_handle, &value[0], sizeof(value));
            } else {
                client->scheduled_tasks |= ASCS_TASK_SEND_VALUE_CHANGED;
                break;
            }
        }
        client->scheduled_tasks &= ~ASCS_TASK_SEND_VALUE_CHANGED;
    }

    if (client->scheduled_tasks != 0){
        client->scheduled_tasks_callback.callback = &audio_stream_control_service_server_can_send_now;
        client->scheduled_tasks_callback.context  = (void*) client;
        att_server_register_can_send_now_callback(&client->scheduled_tasks_callback, client->con_handle);
    }
}

static void ascs_schedule_task(ascs_remote_client_t * client, ascs_streamendpoint_t * streamendpoint, uint8_t task){
    if (client->con_handle == HCI_CON_HANDLE_INVALID){
        client->scheduled_tasks &= ~task;
            
        uint8_t i;
        for (i = 0; i < ASCS_STREAMENDPOINTS_MAX_NUM; i++){
            streamendpoint->value_changed = false;
            streamendpoint->control_point_addressed = false;
            streamendpoint->control_point_response_code = 0;
            streamendpoint->control_point_reason = 0;
        }
        return;
    }

    uint16_t scheduled_tasks = client->scheduled_tasks;
    client->scheduled_tasks |= task;

    if (scheduled_tasks == 0){
        client->scheduled_tasks_callback.callback = &audio_stream_control_service_server_can_send_now;
        client->scheduled_tasks_callback.context  = (void*) client;
        att_server_register_can_send_now_callback(&client->scheduled_tasks_callback, client->con_handle);
    }
}

static bool ascs_codec_configuration_supported(lea_codec_id_t codec_id, uint8_t * codec_config, uint8_t condec_length){
    UNUSED(codec_config);
    UNUSED(condec_length);
    return (codec_id.coding_format == HCI_AUDIO_CODING_FORMAT_LC3);
}

static int audio_stream_control_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);
    
    uint8_t response_code = ASCS_ERROR_CODE_SUCCESS;

    if (attribute_handle == ascs_ase_control_point_handle){
        // write without response
        if (buffer_size < 1){
            return ASCS_ERROR_CODE_UNSUPPORTED_OPCODE;
        }

        uint8_t offset = 0;
        ascs_opcode_t opcode = (ascs_opcode_t)buffer[offset++];
        uint8_t ase_num = buffer[offset++];

        if (ase_num < 1){
            return ASCS_ERROR_CODE_INVALID_LENGTH;
        }

        ascs_remote_client_t * client = ascs_get_remote_client_for_con_handle(con_handle);
        if (client == NULL){
            return ASCS_ERROR_CODE_UNSPECIFIED_ERROR;
        }

        if (ase_num == 0 || ase_num > ASCS_STREAMENDPOINTS_MAX_NUM){
            return ASCS_ERROR_CODE_INVALID_LENGTH;
        }

        uint8_t ase_counter = 0;
        ascs_streamendpoint_t * streamendpoint;

        // if multiple Response_Code and Reason values are applicable to ASE array, 
        // the server shall use the Response_Code and Reason values that apply to the error caused by 
        // the first erroneous ASE in the array as return value of write callback

        switch (opcode){
            case ASCS_OPCODE_CONFIG_CODEC:
                if (ascs_get_client_codec_config_operation_length(ase_num, &buffer[offset], buffer_size - offset) == 0){
                    return ASCS_ERROR_CODE_INVALID_LENGTH;
                }

                for (ase_counter = 0; ase_counter < ase_num; ase_counter++){
                    // check if ASE in valid state
                    uint8_t ase_id = buffer[offset++];
                    streamendpoint = ascs_get_streamendpoint_for_ase_id(client, ase_id);
                    if (streamendpoint == NULL){
                        ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_ASE_ID, 0, &response_code);
                        break;
                    }

                    streamendpoint->control_point_response_code = ASCS_ERROR_CODE_SUCCESS;
                    streamendpoint->control_point_reason = 0;
                    streamendpoint->control_point_addressed = true;

                    lea_client_target_latency_t target_latency;
                    lea_client_target_phy_t target_phy;
                    lea_codec_id_t codec_id;
                    uint8_t   codec_config_lenght;
                    uint8_t * codec_config;

                    switch (streamendpoint->state){
                        case ASCS_STATE_IDLE:
                        case ASCS_STATE_CODEC_CONFIGURED:
                        case ASCS_STATE_QOS_CONFIGURED:
                            target_latency = (lea_client_target_latency_t)buffer[offset++];
                            if (target_latency == LEA_CLIENT_TARGET_LATENCY_NO_PREFERENCE || target_latency >= LEA_CLIENT_TARGET_LATENCY_RFU){
                                ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_MAX_TRANSPORT_LATENCY, &response_code);
                                break;
                            }

                            target_phy = (lea_client_target_phy_t)buffer[offset++];
                            if (target_phy == LEA_CLIENT_TARGET_PHY_NO_PREFERENCE || target_phy >= LEA_CLIENT_TARGET_PHY_RFU){
                                ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_PHY, &response_code);
                                break;
                            }

                            codec_id.coding_format = (hci_audio_coding_format_t)buffer[offset++];
                            if (codec_id.coding_format >= HCI_AUDIO_CODING_FORMAT_RFU &&
                                codec_id.coding_format !=  HCI_AUDIO_CODING_FORMAT_VENDOR_SPECIFIC){
                                ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_CODEC_ID, &response_code);
                                break;
                            }

                            codec_id.company_id = little_endian_read_16(buffer, offset);
                            offset += 2;
                            if (codec_id.coding_format != HCI_AUDIO_CODING_FORMAT_VENDOR_SPECIFIC){
                                if (codec_id.company_id != 0){
                                    ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_CODEC_ID, &response_code);
                                    break;
                                }
                            }

                            codec_id.vendor_specific_codec_id = little_endian_read_16(buffer, offset);
                            offset += 2;
                            if (codec_id.coding_format != HCI_AUDIO_CODING_FORMAT_VENDOR_SPECIFIC){
                                if (codec_id.vendor_specific_codec_id != 0){
                                    ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_CODEC_ID, &response_code);
                                    break;
                                }
                            }
                            
                            codec_config_lenght = buffer[offset++];
                            if (codec_config_lenght > LEA_MAX_CODEC_CONFIG_SIZE){
                                ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_REJECTED_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_CODEC_SPECIFIC_CONFIGURATION, &response_code);
                                break;
                            }
                            
                            codec_config = &buffer[offset];
                            if (!ascs_codec_configuration_supported(codec_id, codec_config, codec_config_lenght)){
                                ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_REJECTED_CONFIGURATION_PARAMETER_VALUE, ASCS_REJECT_REASON_CODEC_SPECIFIC_CONFIGURATION, &response_code);
                                break;
                            }

                            streamendpoint->codec_configuration_target_phy = target_phy;
                            streamendpoint->codec_configuration_target_latency = target_latency;
                            streamendpoint->codec_configuration_codec_id = codec_id;
                            streamendpoint->codec_configuration_codec_tlv_length = codec_config_lenght;
                            memcpy(streamendpoint->codec_configuration_codec_tlv, codec_config, streamendpoint->codec_configuration_codec_tlv_length);
                            break;
                    
                        default:
                            ascs_update_response_code(streamendpoint, ASCS_ERROR_CODE_INVALID_ASE_STATE_MACHINE_TRANSITION, 0, &response_code);
                            break;
                    }
                    // schedule opcode answer via notification
                    // then wait for server to set the resto of codec configuration values, 
                    // and then notify client on each ASE changed sepparately 
                    ascs_schedule_task(client, streamendpoint, ASCS_TASK_SEND_CODEC_CONFIGURED_CONTROL_POINT_RESPONSE);
                }
                break;
            case ASCS_OPCODE_CONFIG_QOS:
                // TODO
                break;
            case ASCS_OPCODE_ENABLE:
                // TODO
                break;
            case ASCS_OPCODE_RECEIVER_START_READY:
                // TODO
                break;
            case ASCS_OPCODE_DISABLE:
                // TODO
                break;
            case ASCS_OPCODE_RECEIVER_STOP_READY:
                // TODO
                break;
            case ASCS_OPCODE_UPDATE_METADATA:
                // TODO
                break;
            case ASCS_OPCODE_RELEASE:
                // TODO
                break;
            case ASCS_OPCODE_RELEASED:
                // TODO
                break;
            default:
                break;
        }

        if (ase_num != ase_counter){
            return ASCS_ERROR_CODE_INVALID_LENGTH;
        }
        if (offset != buffer_size){
            return ASCS_ERROR_CODE_INVALID_LENGTH;
        }
    }

    // else if (attribute_handle == ascs_sink_ase_client_configuration_handle){
    //     ascs_sink_ase_client_configuration = little_endian_read_16(buffer, 0);
    //     ascs_set_con_handle(con_handle, ascs_sink_ase_client_configuration);
    // }

    // else if (attribute_handle == ascs_source_ase_client_configuration_handle){
    //     ascs_source_ase_client_configuration = little_endian_read_16(buffer, 0);
    //     ascs_set_con_handle(con_handle, ascs_source_ase_client_configuration);
    // }
    return response_code;
}

static void audio_stream_control_service_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
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
            
            break;
        default:
            break;
    }
}

static void ascs_streamenpoint_init(
    const uint8_t streamendpoint_characteristics_num, ascs_streamendpoint_characteristic_t * streamendpoint_characteristics, 
    uint16_t start_handle, uint16_t end_handle, ascs_role_t role){

    uint16_t chr_uuid16 = ORG_BLUETOOTH_CHARACTERISTIC_SINK_ASE;
    if (role == ASCS_ROLE_SOURCE){
        chr_uuid16 = ORG_BLUETOOTH_CHARACTERISTIC_SOURCE_ASE;
    }

    // search streamendpoints
    while ( (start_handle < end_handle) && (ascs_streamendpoint_chr_num < streamendpoint_characteristics_num)) {
        uint16_t chr_value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, chr_uuid16);
        uint16_t chr_client_configuration_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, chr_uuid16);
        
        if (chr_value_handle == 0){
            break;
        }

        btstack_assert(ascs_streamendpoint_chr_num < ASCS_STREAMENDPOINTS_MAX_NUM);

        ascs_streamendpoint_characteristic_t * streamendpoint_chr = &ascs_streamendpoint_characteristics[ascs_streamendpoint_chr_num];
        memset(streamendpoint_chr, 0, sizeof(ascs_streamendpoint_characteristic_t));

        streamendpoint_chr->role = role;
        streamendpoint_chr->ase_id = ascs_get_next_streamendpoint_chr_id();

        streamendpoint_chr->value_handle = chr_value_handle;
        streamendpoint_chr->client_configuration_handle = chr_client_configuration_handle;
        streamendpoint_chr->client_configuration = 0;
        
        start_handle = chr_client_configuration_handle + 1;
        ascs_streamendpoint_chr_num++;
    }
}

void audio_stream_control_service_server_init(
        const uint8_t streamendpoint_characteristics_num, ascs_streamendpoint_characteristic_t * streamendpoint_characteristics, 
        const uint8_t clients_num, ascs_remote_client_t * clients){

    btstack_assert(streamendpoint_characteristics_num != 0);
    btstack_assert(clients_num != 0);

    // get service handle range
    uint16_t start_handle = 0;
    uint16_t end_handle   = 0xffff;
    int service_found = gatt_server_get_handle_range_for_service_with_uuid16(ORG_BLUETOOTH_SERVICE_AUDIO_STREAM_CONTROL_SERVICE, &start_handle, &end_handle);
    btstack_assert(service_found != 0);
    UNUSED(service_found);

    ascs_streamendpoint_chr_num = 0;
    ascs_streamendpoint_characteristics_id_counter = 0;
    ascs_streamendpoint_characteristics = streamendpoint_characteristics;

    ascs_streamenpoint_init(streamendpoint_characteristics_num, streamendpoint_characteristics, start_handle, end_handle, ASCS_ROLE_SINK);
    ascs_streamenpoint_init(streamendpoint_characteristics_num, streamendpoint_characteristics, start_handle, end_handle, ASCS_ROLE_SOURCE);

    ascs_clients_num = clients_num;
    ascs_clients = clients;
    memset(ascs_clients, 0, sizeof(ascs_remote_client_t) * ascs_clients_num);
    uint8_t i;
    for (i = 0; i < ascs_clients_num; i++){
        uint8_t j;
        for (j = 0; j < ASCS_STREAMENDPOINTS_MAX_NUM; j++){
            ascs_clients[i].streamendpoints[j].state = ASCS_STATE_UNASSIGNED;
        }
    }

    ascs_ase_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_ASE_CONTROL_POINT);;
    ascs_ase_control_point_client_configuration_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_ASE_CONTROL_POINT);
    log_info("Found ASCS service 0x%02x-0x%02x", start_handle, end_handle);

    // register service with ATT Server
    audio_stream_control_service.start_handle   = start_handle;
    audio_stream_control_service.end_handle     = end_handle;
    audio_stream_control_service.read_callback  = &audio_stream_control_service_read_callback;
    audio_stream_control_service.write_callback = &audio_stream_control_service_write_callback;
    audio_stream_control_service.packet_handler = audio_stream_control_service_packet_handler;
    att_server_register_service_handler(&audio_stream_control_service);
}

void audio_stream_control_service_server_register_packet_handler(btstack_packet_handler_t callback){
    btstack_assert(callback != NULL);
    ascs_event_callback = callback;
}

void audio_stream_control_service_server_configure_codec(hci_con_handle_t client_con_handle, uint8_t ase_id, ascs_codec_configuration_t codec_configuration){
    ascs_remote_client_t * client = ascs_get_remote_client_for_con_handle(client_con_handle);
    if (client == NULL){
        return;
    }

    ascs_streamendpoint_t * streamendpoint = ascs_get_streamendpoint_for_ase_id(client, ase_id);
    if (streamendpoint == NULL){
        return;
    }
    // The server shall not send a notification of the ASE Control Point characteristic value when 
    // completing an autonomously-initiated ASE Control operation.
    memcpy(&streamendpoint->codec_configuration, &codec_configuration, sizeof(ascs_codec_configuration_t));

    if (streamendpoint->w4_server_confirmation){
        streamendpoint->w4_server_confirmation = false;
        streamendpoint->value_changed = true;
        ascs_schedule_task(client, streamendpoint, ASCS_TASK_SEND_VALUE_CHANGED);
    }
}

void audio_stream_control_service_server_deinit(void){
}
