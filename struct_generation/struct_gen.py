#! /usr/bin/env python3

import struct
#from difflib import SequenceMatcher
from os.path import commonprefix
import json

switch_struct_format = '<IIBBIIBIII'

struct_values = []

with open('./rawbytes.bin', 'rb') as f:
    raw_bytes = f.read(struct.calcsize(switch_struct_format))
    while raw_bytes:
         struct_values.append(list(struct.unpack(switch_struct_format, raw_bytes)))
         raw_bytes = f.read(struct.calcsize(switch_struct_format))

import numpy
switch_data = numpy.genfromtxt('switch_table.csv', delimiter=';', dtype=None)
switch_names = [switch_str.decode() for switch_str in switch_data['f1']]

data = numpy.genfromtxt('switch_translation.csv', delimiter=';', dtype=None)


if len(struct_values) != len(switch_names):
    print('Something went wrong...')
    exit(1)

translations = []
for k in range(len(struct_values)):
    translations.append([])
    translation_address = struct_values[k][7]
    translation_count = struct_values[k][6]
    if struct_values[k][5] == 1:
        translation_count += 1 # continuous switch type, but should have the max translation for a valid ptr
    for m in range(translation_count):
        search_address = translation_address + m * 10
        found = None
        for line in data:
            if search_address == int(line[0], 16):
                found = line
                break
        if found is None:
            print('Failed')
        else:
            translations[-1].append(line)

# make them unique
unique_translations = []
generated_set = set()
for translation in translations:
    if translation[0][0] not in generated_set:
        unique_translations.append(translation)
        generated_set.add(translation[0][0])

print(len(unique_translations), len(translations))

id_map = {}
for unique_translation in unique_translations:
    for sub_trans in unique_translation:
        id_map[int(sub_trans[1])] = sub_trans[2].decode()

max_id = max(id_map.keys())
for id_ in range(max_id + 1):
    if id_ not in id_map:
        print('Missing ID in id_map', id_)

translation_addresses = [struct_value[7] for struct_value in struct_values]

translation_names = {}
remaining_translations = []
for unique_translation in unique_translations:
    state_names = [sub_trans[2].decode() for sub_trans in unique_translation]
    common = commonprefix(state_names)
    if common == 'HSD_STA_':
        new_name = common + '_'.join([n.replace(common, '') for n in state_names])
        if new_name not in translation_names.values():
            translation_names[int(unique_translation[0][0], 16)] = new_name
        else:
            k = 1
            while new_name + '_' + str(k) in translation_names.values():
                k += 1
            translation_names[int(unique_translation[0][0], 16)] = new_name + '_' + str(k)
    elif common == 'HSD_STA_O':
        common = common[:-1]
        new_name = common + '_'.join([n.replace(common, '') for n in state_names])
        if new_name not in translation_names.values():
            translation_names[int(unique_translation[0][0], 16)] = new_name
        else:
            k = 1
            while new_name + '_' + str(k) in translation_names.values():
                k += 1
            translation_names[int(unique_translation[0][0], 16)] = new_name + '_' + str(k)
            #print(commonprefix(state_names), state_names, unique_translation)
    elif 'HSD_STA_REF' in common or 'HSD_STA_FE' in common or 'HSD_STA_CBT' in common \
         or 'HSD_STA_COPRO' in common or 'HSD_STA_TRIGGER' in common \
         or 'HSD_STA_MAC' in common or 'HSD_STA_AUDIO' in common:
        translation_names[int(unique_translation[0][0], 16)] = common[:-1]
    elif 'HSD_STA_USU' in common:
        if common[:-1] in translation_names.values():
            translation_names[int(unique_translation[0][0], 16)] = common[:-1] + '_1'
        else:
            translation_names[int(unique_translation[0][0], 16)] = common[:-1]
    elif translation_addresses.count(int(unique_translation[0][0], 16)) == 1:
        idx = translation_addresses.index(int(unique_translation[0][0], 16))
        translation_names[int(unique_translation[0][0], 16)] = switch_names[idx]
        #print('HERE', switch_names[idx], state_names)
    else:
        remaining_translations.append([common, unique_translation])

rem_common_names = [rem_trans[0] for rem_trans in remaining_translations]
for rem_trans in remaining_translations:
    if rem_common_names.count(rem_trans[0]) == 1:
        if rem_trans[0][-1] == '_':
            name = rem_trans[0][:-1]
        else:
            name = rem_trans[0]
        translation_names[int(rem_trans[1][0][0], 16)] = name
    else:
        if int(rem_trans[1][0][0], 16) == 0x47dde8:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_PRG_RUN"
        elif int(rem_trans[1][0][0], 16) == 0x47de00:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RESET_RUN"
        elif int(rem_trans[1][0][0], 16) == 0x47de18:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIF_PRG_RUN_1"
        elif int(rem_trans[1][0][0], 16) == 0x47de30:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RESET_RUN_1"
        elif int(rem_trans[1][0][0], 16) == 0x47e678:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_IQIF_RX_IF_IN"
        elif int(rem_trans[1][0][0], 16) == 0x47e690:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_IQIF_RX_DEMOD_IN"
        elif int(rem_trans[1][0][0], 16) == 0x47e6a8:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_IQIF_TX_IF_IN"
        elif int(rem_trans[1][0][0], 16) == 0x47e6c0:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_IQIF_TX_MOD_IN"
        elif int(rem_trans[1][0][0], 16) == 0x47df18:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_ADC_CAL"
        elif int(rem_trans[1][0][0], 16) == 0x47df30:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_ADC_CONV"
        elif int(rem_trans[1][0][0], 16) == 0x47df60:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_FM_ENABLE"
        elif int(rem_trans[1][0][0], 16) == 0x47dfd8:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_IQ_SRC_AUC_MUX"
        elif int(rem_trans[1][0][0], 16) == 0x47e030:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_AUC_FILT_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e080:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_AUC_IQ_INV_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e098:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_TX_DSP_SAMPLE_CLK"
        elif int(rem_trans[1][0][0], 16) == 0x47e0c8:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_LH_1_SRC_ROUTE"
        elif int(rem_trans[1][0][0], 16) == 0x47e0e0:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_LH_2_SRC_ROUTE"
        elif int(rem_trans[1][0][0], 16) == 0x47e0f8:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_LH_CLK_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e110:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_DUC_1_SRC_ROUTE"
        elif int(rem_trans[1][0][0], 16) == 0x47e128:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_DUC_2_SRC_ROUTE"
        elif int(rem_trans[1][0][0], 16) == 0x47e140:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_NOT_QUIT_QUIT"
        elif int(rem_trans[1][0][0], 16) == 0x47e240:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_NOT_READY_DONE"
        elif int(rem_trans[1][0][0], 16) == 0x47e258:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_INIT_NOT_INIT"
        elif int(rem_trans[1][0][0], 16) == 0x47e270:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_BUSY_NOT_BUSY"
        elif int(rem_trans[1][0][0], 16) == 0x47d998:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_PENDING_NOT_PENDING"
        elif int(rem_trans[1][0][0], 16) == 0x47e290:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_READY_BUSY"
        elif int(rem_trans[1][0][0], 16) == 0x47e2c0:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_LH_ENABLE"
        elif int(rem_trans[1][0][0], 16) == 0x47e2d8:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_LH_DISABLE_ENABLE"
        elif int(rem_trans[1][0][0], 16) == 0x47e350:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RX_SAMPLETRIG_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e638:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_IQIF_RX_MOD_FILT_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e658:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_IQIF_TX_MOD_FILT_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47eb48:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_EXTBUS_SRC_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47ec28:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_EXTBUS_DELAY_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47ec50:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RXDSP_RBUS_TRIG_SRC_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47ed10:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_LH_TRIG_SRC_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e3a0:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_SER_DISABLE_ENABLE"
        elif int(rem_trans[1][0][0], 16) == 0x47e400:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_SER_LH1_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e420:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_SER_LH2_SEL"
        elif int(rem_trans[1][0][0], 16) == 0x47e2f0:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RX_HOST_FIFO_EMPTY"
        elif int(rem_trans[1][0][0], 16) == 0x47e308:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RX_HOST_FIFO_ALMOST_EMPTY"
        elif int(rem_trans[1][0][0], 16) == 0x47e320:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RX_HOST_FIFO_ALMOST_FULL"
        elif int(rem_trans[1][0][0], 16) == 0x47e338:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_RX_HOST_FIFO_FULL"
        elif int(rem_trans[1][0][0], 16) == 0x47de48:
            translation_names[int(rem_trans[1][0][0], 16)] = "HSD_STA_DIG_SPECIAL_CLK_SRC"

        else:
            print(rem_trans)
            idx = 0
            for k in range(translation_addresses.count(int(rem_trans[1][0][0], 16))):
                idx = translation_addresses.index(int(rem_trans[1][0][0], 16), idx+1)
                print(switch_names[idx])
            print()

#print(translation_names)

with open('./streams.json', 'r') as f:
    stream_data = json.load(f)

with open('hw_interface.cpp', 'w') as f:
    f.write('#include "hw_interface.hpp"\n\n')

    f.write('namespace hw_interface {\n\n')

    for unique_translation in unique_translations:
        vec_name = translation_names[int(unique_translation[0][0], 16)]
        f.write(f'static const std::vector<switch_translation> {vec_name.lower()} = {{\n')
        for sub_trans in unique_translation:
            name = sub_trans[2].decode()
            f.write(f'  {{ "{name}", 0x{int(sub_trans[1]):08X}, 0x{int(sub_trans[3]):04X} }},\n')
        f.write('};\n\n')

    f.write('const std::vector<switch_info> switch_infos = {\n')
    for k in range(len(struct_values)):
        translation_vec_name = translation_names[struct_values[k][7]].lower()
        stream_name = stream_data[struct_values[k][4]][1]
        if struct_values[k][5] == 0:
            switch_type = 'DISCRETE'
        else:
            switch_type = 'CONTINUOUS'
        f.write(f'  {{ {switch_names[k]}, "{switch_names[k]}", {struct_values[k][2]}, {struct_values[k][3]},\n'
                f'     {stream_name}, {switch_type}, {struct_values[k][6]},\n'
                f'     &{translation_vec_name}, {struct_values[k][8]}, {struct_values[k][9]} }},\n')
    f.write('};\n\n\n')

    f.write('const std::vector<stream_info> stream_infos = {\n')
    for k in range(len(stream_data)):
        if stream_data[k][0] != k:
            raise RuntimeError("Should not occur {:} and {:}.".format(stream_data[k][0], k))
        interface_mode = 'ERROR'
        address = 'ERROR'
        access_width = 'ERROR'
        if stream_data[k][6] == 1:
            interface_mode = 'IO_PORTS'
            address = stream_data[k][7]
        elif stream_data[k][6] == 2:
            interface_mode = 'MMIO'
            address = stream_data[k][9] * 0x10 + stream_data[k][10]
        elif stream_data[k][6] == 5:
            interface_mode = 'I2C'
            address = stream_data[k][7]
        elif stream_data[k][6] == 6:
            interface_mode = 'THREE_WIRE'
            address = 0
        if stream_data[k][11] == 0:
            access_width = 'BYTE'
        elif stream_data[k][11] == 1:
            access_width = 'WORD'
        elif stream_data[k][11] == 2:
            access_width = 'DWORD'

        default_value = stream_data[k][5] if stream_data[k][5] is not None else 0
        f.write(f'  {{ {stream_data[k][1]}, "{stream_data[k][1]}", {stream_data[k][3]}, {stream_data[k][4]}, 0x{default_value:04X},\n'
                f'     {interface_mode}, 0x{address:04X}, {access_width} }},\n')
    f.write('};')

    f.write('}; // namespace hw_interface')

with open("hw_interface.hpp", "w") as f:
    f.write('#pragma once\n\n')
    f.write('#include <vector>\n#include <cstdint>\n\n\n')

    f.write('namespace hw_interface {\n\n')

    f.write('enum switch_id : uint32_t;\nenum stream_id : uint32_t;\nstruct switch_translation;\n\n')

    f.write('enum switch_type {\n'
            'DISCRETE,\n'
            'CONTINUOUS\n'
            '};\n\n\n')

    f.write('struct switch_info {\n'
            '  enum switch_id switch_id;\n'
            '  char const *name;\n'
            '  uint8_t bit_length;\n'
            '  uint8_t bit_position;\n'
            '  enum stream_id stream_id;\n'
            '  enum switch_type type;\n'
            '  uint8_t state_max_count;\n'
            '  const std::vector<switch_translation> *translation;\n'
            '  uint32_t continuous_min;\n'
            '  uint32_t continuous_max;\n'
            '};\n\n\n')

    f.write('struct switch_translation {\n'
            '  char const *name;\n'
            '  uint32_t id;\n'
            '  uint16_t bitstream_value;\n'
            '};\n\n\n')

    f.write('enum interface_mode {\n'
            '  IO_PORTS,\n'   # 0x01
            '  MMIO,\n'       # 0x02
            '  I2C,\n'        # 0x05
            '  THREE_WIRE\n'  # 0x06
            '};\n\n\n')

    f.write('enum access_width {\n'
            '  BYTE,\n'   # 0x00
            '  WORD,\n'   # 0x01
            '  DWORD,\n'  # 0x02
            '};\n\n\n')

    f.write('struct stream_info {\n'
            '  enum stream_id stream_id;\n'
            '  char const *name;\n'
            '  uint8_t flag;\n'
            '  uint8_t length;\n'
            '  uint32_t data;\n'
            '  enum interface_mode interface_mode;\n'
            '  uint32_t address;\n'
            '  enum access_width access_width;\n'
            '};\n\n\n')

    f.write('extern const std::vector<switch_info> switch_infos;\n'
            'extern const std::vector<stream_info> stream_infos;\n')

    f.write('enum switch_id : uint32_t {\n')
    for line in switch_data:
        f.write('  ' + line['f1'].decode() + '=' + str(line['f0']) + ',\n')
    f.write('};\n\n\n')

    f.write('enum stream_id : uint32_t {\n')
    for line in stream_data:
        f.write('  ' + line[1] + '=' + str(line[0]) + ',\n')
    f.write('};\n\n')

    f.write('}; // namespace hw_interface')
