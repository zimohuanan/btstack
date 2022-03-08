CC = g++

# Requirements: cpputest.github.io

BTSTACK_ROOT =  ../..

CFLAGS  = -DUNIT_TEST -x c++ -g -Wall -Wnarrowing -Wconversion-null 
CFLAGS += -I../

CFLAGS += -I${BTSTACK_ROOT}/src
CFLAGS += -I${BTSTACK_ROOT}/test/mock
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/rijndael

VPATH += ${BTSTACK_ROOT}/src
VPATH += ${BTSTACK_ROOT}/src/ble 
VPATH += ${BTSTACK_ROOT}/src/ble/gatt-service
VPATH += ${BTSTACK_ROOT}/platform/posix
VPATH += ${BTSTACK_ROOT}/3rd-party/rijndael
VPATH += ${BTSTACK_ROOT}/test/mock

COMMON = \
	ad_parser.c                 \
	att_db.c 				 	\
	btstack_linked_list.c       \
	btstack_memory.c            \
	btstack_memory_pool.c       \
	btstack_util.c              \
	hci_dump.c                  \
	hids_device.c               \
	mock_att_server.c           \
	

CFLAGS_COVERAGE = ${CFLAGS} -Ibuild-coverage -fprofile-arcs -ftest-coverage 
CFLAGS_ASAN     = ${CFLAGS} -Ibuild-asan     -fsanitize=address -DHAVE_ASSERT

LDFLAGS += -lCppUTest -lCppUTestExt
LDFLAGS_COVERAGE = ${LDFLAGS} -fprofile-arcs -ftest-coverage
LDFLAGS_ASAN     = ${LDFLAGS} -fsanitize=address

COMMON_OBJ_COVERAGE = $(addprefix build-coverage/,$(COMMON:.c=.o))
COMMON_OBJ_ASAN     = $(addprefix build-asan/,    $(COMMON:.c=.o))


all: build-coverage/battery_service_test build-asan/battery_service_test 						\
	 build-coverage/device_information_service_test build-asan/device_information_service_test 	\
	 build-coverage/tx_power_service_test build-asan/tx_power_service_test 	\
	 build-coverage/bond_management_service_test build-asan/bond_management_service_test 	\
	 build-coverage/microphone_control_service_test build-asan/microphone_control_service_test 	\
	 build-coverage/published_audio_capabilities_service_test build-asan/published_audio_capabilities_service_test 	\
	 build-coverage/broadcast_audio_scan_service_test build-asan/broadcast_audio_scan_service_test 	\
	 build-coverage/audio_stream_control_service_test build-asan/audio_stream_control_service_test 	\

build-%:
	mkdir -p $@

build-%/battery_service_test.h: battery_service_test.gatt | build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/device_information_service_test.h: device_information_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/tx_power_service_test.h: tx_power_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/bond_management_service_test.h: bond_management_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/microphone_control_service_test.h: microphone_control_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/published_audio_capabilities_service_test.h: published_audio_capabilities_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/broadcast_audio_scan_service_test.h: broadcast_audio_scan_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-%/audio_stream_control_service_test.h: audio_stream_control_service_test.gatt| build-%
	python3 ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@

build-coverage/%.o: %.c | build-coverage
	${CC} -c $(CFLAGS_COVERAGE) $< -o $@

build-coverage/%.o: %.cpp | build-coverage
	${CC} -c $(CFLAGS_COVERAGE) $< -o $@

build-asan/%.o: %.c | build-asan
	${CC} -c $(CFLAGS_ASAN) $< -o $@

build-asan/%.o: %.cpp | build-asan
	${CC} -c $(CFLAGS_ASAN) $< -o $@

build-coverage/battery_service_test: build-coverage/battery_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/battery_service_server.o build-coverage/battery_service_test.o | build-coverage
	${CC} $(filter-out build-coverage/battery_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/battery_service_test: build-asan/battery_service_test.h ${COMMON_OBJ_ASAN} build-asan/battery_service_server.o build-asan/battery_service_test.o | build-asan
	${CC} $(filter-out build-asan/battery_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/device_information_service_test: build-coverage/device_information_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/device_information_service_server.o build-coverage/device_information_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/device_information_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/device_information_service_test: build-asan/device_information_service_test.h ${COMMON_OBJ_ASAN} build-asan/device_information_service_server.o build-asan/device_information_service_test.o  | build-asan
	${CC} $(filter-out build-asan/device_information_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/tx_power_service_test: build-coverage/tx_power_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/tx_power_service_server.o build-coverage/tx_power_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/tx_power_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/tx_power_service_test: build-asan/tx_power_service_test.h ${COMMON_OBJ_ASAN} build-asan/tx_power_service_server.o build-asan/tx_power_service_test.o  | build-asan
	${CC} $(filter-out build-asan/tx_power_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/bond_management_service_test: build-coverage/bond_management_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/bond_management_service_server.o build-coverage/bond_management_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/bond_management_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/bond_management_service_test: build-asan/bond_management_service_test.h ${COMMON_OBJ_ASAN} build-asan/bond_management_service_server.o  build-asan/bond_management_service_test.o  | build-asan
	${CC} $(filter-out build-asan/bond_management_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/microphone_control_service_test: build-coverage/microphone_control_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/audio_input_control_service_server.o build-coverage/microphone_control_service_server.o build-coverage/microphone_control_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/microphone_control_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/microphone_control_service_test: build-asan/microphone_control_service_test.h ${COMMON_OBJ_ASAN} build-asan/audio_input_control_service_server.o build-asan/microphone_control_service_server.o build-asan/microphone_control_service_test.o  | build-asan
	${CC} $(filter-out build-asan/microphone_control_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/published_audio_capabilities_service_test: build-coverage/published_audio_capabilities_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/published_audio_capabilities_service_server.o build-coverage/published_audio_capabilities_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/published_audio_capabilities_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/published_audio_capabilities_service_test: build-asan/published_audio_capabilities_service_test.h ${COMMON_OBJ_ASAN} build-asan/published_audio_capabilities_service_server.o  build-asan/published_audio_capabilities_service_test.o  | build-asan
	${CC} $(filter-out build-asan/published_audio_capabilities_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/broadcast_audio_scan_service_test: build-coverage/broadcast_audio_scan_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/broadcast_audio_scan_service_server.o build-coverage/broadcast_audio_scan_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/broadcast_audio_scan_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/broadcast_audio_scan_service_test: build-asan/broadcast_audio_scan_service_test.h ${COMMON_OBJ_ASAN} build-asan/broadcast_audio_scan_service_server.o  build-asan/broadcast_audio_scan_service_test.o  | build-asan
	${CC} $(filter-out build-asan/broadcast_audio_scan_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

build-coverage/audio_stream_control_service_test: build-coverage/audio_stream_control_service_test.h ${COMMON_OBJ_COVERAGE} build-coverage/audio_stream_control_service_server.o build-coverage/audio_stream_control_service_test.o  | build-coverage
	${CC} $(filter-out build-coverage/audio_stream_control_service_test.h,$^) ${LDFLAGS_COVERAGE} -o $@

build-asan/audio_stream_control_service_test: build-asan/audio_stream_control_service_test.h ${COMMON_OBJ_ASAN} build-asan/audio_stream_control_service_server.o  build-asan/audio_stream_control_service_test.o  | build-asan
	${CC} $(filter-out build-asan/audio_stream_control_service_test.h,$^) ${LDFLAGS_ASAN} -o $@

test: all
	build-asan/battery_service_test
	build-asan/device_information_service_test
	build-asan/tx_power_service_test
	build-asan/bond_management_service_test
	build-asan/microphone_control_service_test
	build-asan/published_audio_capabilities_service_test
	build-asan/broadcast_audio_scan_service_test
	build-asan/audio_stream_control_service_test
		
coverage: all
	rm -f build-coverage/*.gcda
	build-coverage/battery_service_test
	build-coverage/device_information_service_test
	build-asan/tx_power_service_test
	build-asan/bond_management_service_test
	build-asan/microphone_control_service_test
	build-asan/published_audio_capabilities_service_test
	build-asan/broadcast_audio_scan_service_test
	build-asan/audio_stream_control_service_test

clean:
	rm -rf build-coverage build-asan

