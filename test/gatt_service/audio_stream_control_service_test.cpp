
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
#include "ble/gatt-service/audio_stream_control_service_server.h"
#include "audio_stream_control_service_test.h"
#include "mock_att_server.h"

#define ASCS_NUM_STREAMENDPOINTS 5
#define ASCS_NUM_CLIENTS 3

static ascs_streamendpoint_t ascs_streamendpoints[ASCS_NUM_STREAMENDPOINTS];
static ascs_remote_client_t ascs_clients[ASCS_NUM_CLIENTS];

TEST_GROUP(AUDIO_STREAM_CONTROL_SERVICE_SERVER){ 
    att_service_handler_t * service; 
    uint16_t con_handle;
    uint16_t ascs_control_point_handle;

    void setup(void){
        // setup database
        att_set_db(profile_data);
        ascs_control_point_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(0, 0xffff, ORG_BLUETOOTH_CHARACTERISTIC_ASE_CONTROL_POINT);
    }

    void teardown(){
        audio_stream_control_service_server_deinit();
        mock_deinit();
    }
};


TEST(AUDIO_STREAM_CONTROL_SERVICE_SERVER, lookup_attribute_handles){
    CHECK(ascs_control_point_handle != 0);
    
    uint16_t expected_streamendpoints_num = 5;
    uint16_t streamendpoints_num = 0;
    
    audio_stream_control_service_server_init(ASCS_NUM_STREAMENDPOINTS, ascs_streamendpoints, ASCS_NUM_CLIENTS, ascs_clients);

    uint16_t i;
    for (i = 0; i < ASCS_NUM_STREAMENDPOINTS; i++){
        ascs_streamendpoint_t streamendpoint = ascs_streamendpoints[i];
        CHECK(streamendpoint.value_handle != 0);
        CHECK(streamendpoint.client_configuration_handle != 0);
        streamendpoints_num++;
    }
    CHECK_EQUAL(expected_streamendpoints_num, streamendpoints_num);
}

int main (int argc, const char * argv[]){
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
