
// *****************************************************************************
//
// test BASS
//
// *****************************************************************************


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CppUTest/TestHarness.h"
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTestExt/MockSupport.h"

#include "hci.h"
#include "gap.h"
#include "btstack_util.h"
#include "bluetooth.h"
#include "bluetooth_gatt.h"
#include "ble/le_device_db.h"
#include "ble/gatt-service/broadcast_audio_scan_service_server.h"
#include "broadcast_audio_scan_service_test.h"
#include "mock_att_server.h"

#define BASS_NUM_SOURCES 2
#define BASS_UNDEFINED_EVENT    0xFF

static bass_source_t sources[2];

static uint8_t expected_scan_active = 0;
static uint8_t received_event = 0;

TEST_GROUP(BROADCAST_AUDIO_SCAN_SERVICE_SERVER){ 
    att_service_handler_t * service; 
    uint16_t con_handle;
    uint16_t bass_audio_scan_control_point_handle;

    void setup(void){
        // setup database
        att_set_db(profile_data);
        bass_audio_scan_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(0, 0xffff, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_AUDIO_SCAN_CONTROL_POINT);

        // con_handle = 0x00;
    }

    void teardown(){
        broadcast_audio_scan_service_server_deinit();
        mock_deinit();
    }
};


TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, lookup_attribute_handles){
    CHECK(bass_audio_scan_control_point_handle != 0);
    
    uint16_t expected_sources_num = 2;
    uint16_t sources_num = 0;
    
    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);

    btstack_linked_list_t * bass_sources = broadcast_audio_scan_service_server_get_sources();

    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        CHECK(item->bass_receive_state_handle != 0);
        CHECK(item->bass_receive_state_client_configuration_handle != 0);
        sources_num++;
    }
    CHECK_EQUAL(expected_sources_num, sources_num);
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, read_receive_state_client_configuration){
    uint8_t  expected_read_buffer[] = {0, 0};
    uint8_t  read_buffer[2];
    
    uint16_t expected_sources_num = 2;
    uint16_t sources_num = 0;

    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);
    btstack_linked_list_t * bass_sources = broadcast_audio_scan_service_server_get_sources();

    // invalid attribute handle
    uint16_t response_len = mock_att_service_read_callback(con_handle, 0xffff, 0xffff, read_buffer, sizeof(read_buffer));
    CHECK_EQUAL(0, response_len);

    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        response_len = mock_att_service_read_callback(con_handle, item->bass_receive_state_client_configuration_handle, 0, read_buffer, sizeof(read_buffer));
        CHECK_EQUAL(2, response_len);
        MEMCMP_EQUAL(expected_read_buffer, read_buffer, sizeof(expected_read_buffer));
        sources_num++;
    }
    CHECK_EQUAL(expected_sources_num, sources_num);
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_receive_state_client_configuration){
    uint8_t  write_buffer[] = {0, 1};
    uint8_t  expected_read_buffer[] = {0, 1};
    uint8_t  read_buffer[2];
    
    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);
    btstack_linked_list_t * bass_sources = broadcast_audio_scan_service_server_get_sources();

    // invalid attribute handle
    uint16_t response_len = mock_att_service_read_callback(con_handle, 0xffff, 0xffff, read_buffer, sizeof(read_buffer));
    CHECK_EQUAL(0, response_len);

    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);

        int response = mock_att_service_write_callback(con_handle, item->bass_receive_state_client_configuration_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
        CHECK_EQUAL(0, response); 

        response_len = mock_att_service_read_callback(con_handle, item->bass_receive_state_client_configuration_handle, 0, read_buffer, sizeof(read_buffer));
        CHECK_EQUAL(2, response_len);
        MEMCMP_EQUAL(expected_read_buffer, read_buffer, sizeof(expected_read_buffer));
    }
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, read_receive_state_source_id){
    uint8_t expected_bass_source[15];
    uint8_t read_buffer[50];
    memset(expected_bass_source, 0, sizeof(expected_bass_source));
    memset(read_buffer, 0, sizeof(read_buffer));

    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);
    btstack_linked_list_t * bass_sources = broadcast_audio_scan_service_server_get_sources();

    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);

        uint16_t response_len = mock_att_service_read_callback(con_handle, item->bass_receive_state_handle, 0, read_buffer, sizeof(read_buffer));
        CHECK_EQUAL(15, response_len);
        CHECK_EQUAL(item->source_id, read_buffer[0]);
        
        expected_bass_source[0] = item->source_id;
        MEMCMP_EQUAL(expected_bass_source, read_buffer, sizeof(expected_bass_source));
    }
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    CHECK_EQUAL(packet_type, HCI_EVENT_PACKET);
    if (packet[0] != HCI_EVENT_GATTSERVICE_META){
        return;
    }
    received_event = 0;

    switch (packet[2]){
    
        case GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STOPED:
            CHECK_EQUAL(expected_scan_active, 0); 
            received_event = GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STOPED;
            break;
        
        case GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STARTED:
            CHECK_EQUAL(expected_scan_active, 1); 
            received_event = GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STARTED;
            break;

        case GATTSERVICE_SUBEVENT_BASS_PA_SYNC_STATE:
            received_event = GATTSERVICE_SUBEVENT_BASS_PA_SYNC_STATE;
            break;

        case GATTSERVICE_SUBEVENT_BASS_BROADCAST_CODE:
            received_event = GATTSERVICE_SUBEVENT_BASS_BROADCAST_CODE;
            break;

        default:
            received_event = BASS_UNDEFINED_EVENT;
            break;
    }

}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, invalid_write_control_point_opcode_not_supported){
    uint8_t write_buffer[1];

    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);

    // empty buffer
    uint16_t write_buffer_size = 0;
    int response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, write_buffer_size);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 

    // wrong opcode
    write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_RFU;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, write_buffer_size);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response);    
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_control_point_scan){
    uint8_t write_buffer[1];
    int response;

    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);

    broadcast_audio_scan_service_server_register_packet_handler(&packet_handler);

    write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_REMOTE_SCAN_STOPPED;
    expected_scan_active = 0;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(0, response); 
    CHECK_EQUAL(GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STOPED, received_event); 

    write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_REMOTE_SCAN_STARTED;
    expected_scan_active = 1;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(0, response); 
    CHECK_EQUAL(GATTSERVICE_SUBEVENT_BASS_REMOTE_SCAN_STARTED, received_event); 
}


TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_control_point_add_source){
    uint8_t write_buffer[50];
    int response;
    bd_addr_t remote_addr = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint32_t broadcast_id = 0xB1B2B3;
    uint16_t pa_interval = 0xCCDD;
    uint32_t bis_sync_state = 0xFFFFFFFF;

    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);
    received_event = 0;

    memset(write_buffer, 0, sizeof(write_buffer));
    
    broadcast_audio_scan_service_server_register_packet_handler(&packet_handler);

    write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_ADD_SOURCE;
    
    // buffer size < 15
    // no event is emitted
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 10);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);
    
    // wrong Advertiser_Address_Type
    write_buffer[1] = 0x03;
    // no event is emitted
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    // Advertiser_Address_Type = Public Device Address 
    write_buffer[1] = 0x00;
    reverse_bd_addr(remote_addr, &write_buffer[2]);
    // Wrong advertising_sid Range: 0x00-0x0F
    write_buffer[8] = 0xFF;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    // advertising_sid Range: 0x01
    write_buffer[8] = 0x01;
    // broadcast_id
    little_endian_store_24(write_buffer, 9, broadcast_id);

    // Wrong pa_sync_state
    write_buffer[12] = (uint8_t)LEA_PA_SYNC_RFU;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    // pa_sync_state LEA_PA_SYNC_SYNCHRONIZE_TO_PA_PAST_AVAILABLE
    write_buffer[12] = (uint8_t)LEA_PA_SYNC_SYNCHRONIZE_TO_PA_PAST_AVAILABLE;
    // pa_interval
    little_endian_store_16(write_buffer, 13, pa_interval);

    // num_subgroups excidees BASS_SUBGROUPS_MAX_NUM
    write_buffer[15] = BASS_SUBGROUPS_MAX_NUM + 1;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    write_buffer[15] = 0;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 16);
    CHECK_EQUAL(0, response); 
    CHECK_EQUAL(GATTSERVICE_SUBEVENT_BASS_PA_SYNC_STATE, received_event); 
    received_event = 0;

    // num_subgroups valid, no metadata_length
    write_buffer[15] = 1;
    little_endian_store_32(write_buffer, 16, bis_sync_state);
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 20);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    // num_subgroups valid, metadata_length exceeds BASS_METADATA_MAX_LENGTH
    write_buffer[20] = BASS_METADATA_MAX_LENGTH + 1;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 22);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    // num_subgroups valid, metadata_length exceeds BASS_METADATA_MAX_LENGTH
    write_buffer[20] = BASS_METADATA_MAX_LENGTH + 1;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 22);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);

    // num_subgroups valid, metadata_length valid, metadata nod complete
    write_buffer[20] = 10;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 28);
    CHECK_EQUAL(ATT_ERROR_WRITE_REQUEST_REJECTED, response); 
    CHECK_EQUAL(0, received_event);
 
    // valid source with subgroup and metadata
    write_buffer[20] = 10;
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 31);
    CHECK_EQUAL(0, response); 
    CHECK_EQUAL(GATTSERVICE_SUBEVENT_BASS_PA_SYNC_STATE, received_event); 
    received_event = 0;


    // write two subgroups
    write_buffer[15] = 2;
    little_endian_store_32(write_buffer, 16, bis_sync_state);
    write_buffer[20] = 10;
    memset(&write_buffer[21], 0xAA, 10);

    little_endian_store_32(write_buffer, 31, bis_sync_state);
    write_buffer[35] = 4;
    memset(&write_buffer[36], 0xBB, 4);
    
    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, 40);
    CHECK_EQUAL(0, response); 
    CHECK_EQUAL(GATTSERVICE_SUBEVENT_BASS_PA_SYNC_STATE, received_event);  
    received_event = 0;
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_control_point_set_broadcast_code){
    uint8_t write_buffer[18];
    int response;

    broadcast_audio_scan_service_server_init(BASS_NUM_SOURCES, sources);

    received_event = 0;
    memset(write_buffer, 0xAA, sizeof(write_buffer));
    broadcast_audio_scan_service_server_register_packet_handler(&packet_handler);

    write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_SET_BROADCAST_CODE;
    write_buffer[1] = 1; // source_id

    response = mock_att_service_write_callback(con_handle, bass_audio_scan_control_point_handle, ATT_TRANSACTION_MODE_NONE, 0, write_buffer, sizeof(write_buffer));
    CHECK_EQUAL(0, response); 
    CHECK_EQUAL(GATTSERVICE_SUBEVENT_BASS_BROADCAST_CODE, received_event);
}


TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_control_point_modify_source){
    // write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_MODIFY_SOURCE;
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_control_point_remove_source){
    // write_buffer[0] = (uint8_t)LEA_BASS_OPCODE_SET_BROADCAST_CODE;
}

int main (int argc, const char * argv[]){
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
