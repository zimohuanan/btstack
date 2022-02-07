
// *****************************************************************************
//
// test battery service
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
#include "btstack_util.h"
#include "bluetooth.h"
#include "bluetooth_gatt.h"

#include "ble/gatt-service/published_audio_capabilities_service_server.h"
#include "published_audio_capabilities_service_test.h"
#include "mock_att_server.h"

const uint8_t  my_value[]    = {0x06, 0x07};
static pacs_codec_specific_capability_t my_capability = {
        LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_SAMPLING_FREQUENCY,
        my_value
    };
const uint8_t my_metadata[] = {0x09, 0x0A, 0x0B, 0x0C};

static pacs_record_t sink_record_0 = {
    // codec ID
    {LE_AUDIO_CODING_FORMAT_LC3, 0xAABB, 0xCCDD},
    // num capabilities
    1, 
    // capabilities
    &my_capability,
    // metadata length
    4, 
    // metadata
    my_metadata
};

uint8_t expected_response_sink_pac_record[] = {
        // num_records
        0x01,
        // codec id
        (uint8_t)LE_AUDIO_CODING_FORMAT_LC3, 0xBB, 0xAA, 0xDD, 0xCC,
        // cap. length, value_len + 1 byte for type, type, value
        0x04, 0x03, (uint8_t)LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_SAMPLING_FREQUENCY, 0x06, 0x07,
        // metadata len, metadata
        0x04, 0x09, 0x0A, 0x0B, 0x0C
};

TEST_GROUP(PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER){ 
    att_service_handler_t * service; 
    uint16_t con_handle;
    uint16_t pacs_sinc_pac_handle_value;
    pacs_record_t sink_pac_records[1];
    uint8_t sink_pac_records_num;
    uint32_t sink_audio_location_bitmap;
    uint32_t source_audio_location_bitmap;

    void setup(void){
        // setup database
        att_set_db(profile_data);
        pacs_sinc_pac_handle_value = gatt_server_get_value_handle_for_characteristic_with_uuid16(0, 0xffff, ORG_BLUETOOTH_CHARACTERISTIC_SINK_PAC);
        sink_pac_records_num = 1;
        sink_pac_records[0] = sink_record_0;
        // setup battery service
        sink_audio_location_bitmap = LEA_AUDIO_LOCATION_FRONT_RIGHT;
        source_audio_location_bitmap = LEA_AUDIO_LOCATION_NOT_ALLOWED;

        published_audio_capabilities_service_server_init(
                sink_pac_records, sink_pac_records_num,
                NULL, 0,
                sink_audio_location_bitmap,
                source_audio_location_bitmap,
                0,0,0,0);

        service = mock_att_server_get_service();
        con_handle = 0x00;
    }

    void teardown(){
        mock_deinit();
    }
};


TEST(PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER, lookup_attribute_handles){
    CHECK(pacs_sinc_pac_handle_value != 0);
}

TEST(PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER, read_pac_handle_client_configuration) {
    uint8_t response[2];
    uint16_t response_len;

    // invalid attribute handle
    response_len = mock_att_service_read_callback(con_handle, 0xffff, 0xffff, response, sizeof(response));
    CHECK_EQUAL(0, response_len);
}

TEST(PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER, read_whole_sink_pac_record) {
    uint8_t  response[20];
    uint16_t response_len = mock_att_service_read_callback(con_handle, pacs_sinc_pac_handle_value, 0, response, sizeof(response));
    CHECK_EQUAL(sizeof(expected_response_sink_pac_record), response_len);
    MEMCMP_EQUAL(expected_response_sink_pac_record, response, sizeof (expected_response_sink_pac_record));
}

TEST(PUBLISHED_AUDIO_CAPABILITIES_SERVICE_SERVER, read_partial_sink_pac_record) {
    uint8_t response[10];
    uint8_t max_response_size = sizeof(response);
    // slide and read max 10 bytes
    uint8_t offset;
    for (offset = 0; offset < sizeof(expected_response_sink_pac_record); offset++){
        uint8_t expected_size = btstack_min(sizeof(expected_response_sink_pac_record) - offset, max_response_size);
        uint16_t response_len = mock_att_service_read_callback(con_handle, pacs_sinc_pac_handle_value, offset, response, max_response_size);
        CHECK_EQUAL(expected_size, response_len);
        MEMCMP_EQUAL(expected_response_sink_pac_record + offset, response, expected_size);
    }
}

int main (int argc, const char * argv[]){
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
