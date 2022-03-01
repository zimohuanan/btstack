
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

        // setup tx power service
        con_handle = 0x00;

    }

    void teardown(){
        mock_deinit();
    }
};

TEST(BROADCAST_AUDIO_SCAN_SERVICE_SERVER, lookup_attribute_handles){
    CHECK(bass_audio_scan_control_point_handle != 0);
    
    btstack_linked_list_iterator_t it;    
    btstack_linked_list_iterator_init(&it, bass_sources);
    while (btstack_linked_list_iterator_has_next(&it)){
        bass_source_t * item = (bass_source_t*) btstack_linked_list_iterator_next(&it);
        CHECK(item->bass_receive_state_handle != 0);
        CHECK(item->bass_receive_state_client_configuration_handle != 0);
    }
}


int main (int argc, const char * argv[]){
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
