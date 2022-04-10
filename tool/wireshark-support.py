#!/usr/bin/env python3

import glob
import re
import fileinput
import sys
import os

import btstack_parser as parser

program_info = '''
BTstack event support for Wireshark
Copyright 2022, BlueKitchen GmbH
'''

# Tags
bk_addition_start = '/* BlueKitchen BTstack Support BEGIN */'
bk_addition_end   = '/* BlueKitchen BTstack Support END */'

# hints
hint_evt_code_vals = '/* Other */'
hint_add_hf_vars = 'static int hf_packet_type_sco_reserved_4_0 = -1;'
hint_add_enums      = 'unit_name_string units_number_events'
hint_dissect_event  = 'case 0xff: /* Vendor-Specific */'
hint_add_header_fields = '{ &hf_bthci_evt_code,'

meta_events = [
    'A2DP',
    'ANCS',
    'AVDTP',
    'AVRCP',
    'GAP',
    'GATTSERVICE',
    'GOEP',
    'HFP',
    'HID',
    'HIDS',
    'HSP',
    'LE',
    'MAP',
    'MESH',
    'PBAP',
]

supported_event_groups = meta_events + [
    'ATT',
    'BNEP',
    'BTSTACK',
    'GAP',
    'GATT',
    'HCI',
    'HID',
    'L2CAP',
    'RFCOMM',
    'SDP',
    'SM'
]


target_file = 'epan/dissectors/packet-bthci_evt.c'

def ws_type_for_btstack_type(type):
    param_types = { 
    # '1' : 'uint8_t', '2' : 'uint16_t', '3' : 'uint32_t', '4' : 'uint32_t', 'H' : 'hci_con_handle_t', 'B' : 'bd_addr_t',
    # 'D' : 'const uint8_t *', 'E' : 'const uint8_t * ', 'N' : 'const char *' , 'P' : 'const uint8_t *', 'A' : 'const uint8_t *',
    # 'R' : 'const uint8_t *', 'S' : 'const uint8_t *',
    # 'J' : 'uint8_t', 'L' : 'uint16_t', 'V' : 'const uint8_t *', 'U' : 'BT_UUID',
    # 'Q' : 'uint8_t *', 'K' : 'uint8_t *',
    # 'X' : 'gatt_client_service_t *', 'Y' : 'gatt_client_characteristic_t *', 'Z' : 'gatt_client_characteristic_descriptor_t *',
    # 'T' : 'const char *', 'C' : 'uint16_t'
    }
    return param_types[type]

def get_fields(events):
    fields = {}
    for event_type, event_name, format, args in events:
        event_type_val = int(event_type, 16)
        if event_type_val < 0x060:
            continue
        for f, arg in zip(format, args):
            typed_arg = arg + "_" + f
            if typed_arg not in fields:
                fields[typed_arg] = f
    return fields


# helper
def insert_hci_events(events):
    for event_type, event_name, format, args in events:
        event_type_val = int(event_type, 16)
        if event_type_val < 0x060:
            continue
        print('    {%s, "%s"},' % (event_type, event_name))

def insert_meta_events(defines):
    for key in defines.keys():
        if key.endswith("META"):
            if key == "HCI_EVENT_LE_META":
                continue
            print('    {%s, "%s"},' % (defines[key], key))

def insert_hf_indices(fields):
    for field in sorted(fields.keys()):
        f = fields[field]
        typed_arg = field + "_" + f
        print('static int %s = -1;' % ("hf_btstack_" + typed_arg))

def insert_hf_declaration(events):
    for field in sorted(fields.keys()):
        f = fields[field]
        typed_arg = field + "_" + f
        # TODO

def insert_dissectors(events):
    for event_type, event_name, format, args in events:
        event_type_val = int(event_type, 16)
        if event_type_val < 0x060:
            continue
        print('        case %s: /* % s */' % (event_type, event_name))
        print('            break;')


# main code
btstack_root = os.path.abspath(os.path.dirname(sys.argv[0]) + '/..')

print(program_info)

# check if target file exists
if not os.path.exists(target_file):
    print("Please run this from your Wireshark root folder" )
    sys.exit(10)

print("[+] Found %s" % target_file)

# parse events
(events, subevents, event_types) = parser.parse_events()

# for event_type, event_name, format, args in events:
#     print(event_type, event_name, format, args)


# read defines from hci_cmds.h and hci.h
defines = parser.parse_defines()

# get field definition
fields = get_fields(events)
print(fields)

drop_old = False
for line in fileinput.input(files=(target_file), inplace=True):
    line = line.rstrip()

    # drop previous changes
    if drop_old:
        if bk_addition_end in line:
            drop_old = False
        continue
    if bk_addition_start in line:
        drop_old = True
        continue

    # insert hf vars - used to store field index
    if hint_add_hf_vars in line:
        print(line)
        print(bk_addition_start)
        # insert_hf_indices(fields)
#         print('''
# static int hf_bthci_hci_state = -1;''')
        print(bk_addition_end)
        continue

    # insert events
    if hint_evt_code_vals in line:
        print(bk_addition_start)
        insert_hci_events(events)
        insert_meta_events(defines)
        print(bk_addition_end)

    # insert enums
    if hint_add_enums in line:
        print(line)
        print(bk_addition_start)
#         print('''
# static const value_string hci_state_vals[] = {
#     { 0x00, "HCI Off" },
#     { 0x01, "HCI Initializing" },
#     { 0x02, "HCI Working" },
#     { 0x03, "HCI Halting" },
#     { 0x04, "HCI Sleeping" },
#     { 0x05, "HCI Falling Asleep" },
#     { 0, NULL }
# };''')
        print(bk_addition_end)
        continue

    # insert dissectors
    if hint_dissect_event in line:
        print(bk_addition_start)
        insert_dissectors(events)
#         print('''
#         case 0x60: /* HCI State */
#             proto_tree_add_item(tree, hf_bthci_hci_state, tvb, offset, 1, ENC_LITTLE_ENDIAN);
#             offset += 1;
#             break;
# ''')

        print(bk_addition_end)

    # insert header fields
    if hint_add_header_fields in line:
        print(bk_addition_start)
        insert_hf_declaration(events)
#         print('''
#         { &hf_bthci_hci_state,
#           { "State", "bthci_evt.hci_state",
#             FT_UINT8, BASE_HEX, VALS(hci_state_vals), 0x0,
#             NULL, HFILL }
#         },
# ''')
        print(bk_addition_end)

    print(line)


