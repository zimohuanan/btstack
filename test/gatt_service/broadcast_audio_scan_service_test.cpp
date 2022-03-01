
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

static bass_source_t sources[2];
static uint8_t sources_num = 2;

TEST_GROUP(BROADCAST_AUDIO_SCAN_SERVICE_SERVER){ 
    att_service_handler_t * service; 
    uint16_t con_handle;
    uint16_t bass_audio_scan_control_point_handle;
    btstack_linked_list_t * bass_sources;

    void setup(void){
        // setup database
        att_set_db(profile_data);
        broadcast_audio_scan_service_server_init(sources_num, sources);

        bass_sources = broadcast_audio_scan_service_server_get_sources();
        bass_audio_scan_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(0, 0xffff, ORG_BLUETOOTH_CHARACTERISTIC_BROADCAST_AUDIO_SCAN_CONTROL_POINT);

        con_handle = 0x00;
    }

    void teardown(){
        mock_deinit();
    }
};

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, lookup_attribute_handles){
    CHECK(bass_audio_scan_control_point_handle != 0);
    
    uint8_t expected_source_id = 2;
    uint16_t expected_sources_num = 2;
    uint16_t sources_num = 0;
    
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        CHECK(item->bass_receive_state_handle != 0);
        CHECK(item->bass_receive_state_client_configuration_handle != 0);
        CHECK_EQUAL(expected_source_id, item->source_id);
        expected_source_id--;
        sources_num++;
    }
    CHECK_EQUAL(expected_sources_num, sources_num);
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, read_receive_state_client_configuration){
    uint8_t  expected_read_buffer[] = {0, 0};
    uint8_t  read_buffer[2];
    
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
    }
}

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, write_receive_state_client_configuration){
    uint8_t  write_buffer[] = {0, 1};
    uint8_t  expected_read_buffer[] = {0, 1};
    uint8_t  read_buffer[2];
    
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

    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);

        uint16_t response_len = mock_att_service_read_callback(con_handle, item->bass_receive_state_handle, 0, read_buffer, sizeof(read_buffer));
        CHECK_EQUAL(15, response_len);
        CHECK_EQUAL(item->source_id, read_buffer[0]);
        
        memset(expected_bass_source, 0, sizeof(expected_bass_source));
        expected_bass_source[0] = item->source_id;
        MEMCMP_EQUAL(expected_bass_source, read_buffer, sizeof(expected_bass_source));
    }
}

int main (int argc, const char * argv[]){
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
