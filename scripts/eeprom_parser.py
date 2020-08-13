#! /usr/bin/env python3

import struct
import binascii
import numpy

class Block(object):
    FORMAT = '<HHH'

    @staticmethod
    def check_checksum(block_data):
        word_array = numpy.frombuffer(block_data, dtype=numpy.uint16)
        return numpy.sum(word_array, dtype=numpy.uint16) == 0

    @staticmethod
    def from_binary(data):
        block_id = 0
        byte_offset = 0
        result = []
        while block_id != 0xFFFF:
            block_id, block_len, _ = struct.unpack('<HHH', bin_data[byte_offset:byte_offset+6])
            if not Block.check_checksum(bin_data[byte_offset:byte_offset+block_len]):
                print('Checksum mismatch')
            payload = bin_data[byte_offset+6:byte_offset+block_len]
            if block_id == 0xFFFF:
                result.append(End(payload, block_id))
            elif block_id >= 2 and block_id <= 99:
                result.append(Ident(payload, block_id))
            elif block_id >= 100 and block_id <= 199:
                result.append(Modification(payload, block_id))
            elif block_id >= 200 and block_id <= 299:
                result.append(Service(payload, block_id))
            elif block_id >= 300 and block_id <= 399:
                result.append(Statistics(payload, block_id))
            elif block_id >= 10000:
                result.append(User(payload, block_id))
            byte_offset += block_len
        return result


class Ident(object):
    FORMAT = '<HIIBBHIHHH14s'

    def __init__(self, data, block_id=2):
        self.block_id = block_id
        self.alignment, self.eeprom_size, self.part_number, self.variant, \
            self.hw_code, self.product_index, self.serial_number, self.product_date, \
            self.read_code, self.test_instruction, self.name = struct.unpack(Ident.FORMAT, data)

    def __repr__(self):
        return 'ID:                 IDENT_{}\n'.format(self.block_id) + \
            'ALIGNMENT:          {}\n'.format(self.alignment) + \
            'EEPROMSIZE:         {}\n'.format(self.eeprom_size) + \
            'PARTNUMBER:         {:04}.{:04}.{:02}\n'.format(int(self.part_number / 10000), int(self.part_number % 10000), self.variant) + \
            'HWCODE:             {}\n'.format(self.hw_code) + \
            'PRODUCTINDEX:       {:02}.{:02}\n'.format(int(self.product_index / 100), int(self.product_index % 100)) + \
            'SN:                 {}/{:03}\n'.format(int(self.serial_number / 1000), int(self.serial_number % 1000)) + \
            'PRODUCTDATE:        {}-{:02}-{:02}\n'.format(int(self.product_date / 512) + 1980,
                                                          int(int(self.product_date % 512) / 32),
                                                          int(int(self.product_date % 512) % 32)) + \
            'READCODE:           {}\n'.format(self.read_code) + \
            'TESTINSTRUCTION:    {:02}.{:02}\n'.format(int(self.test_instruction / 100), int(self.test_instruction % 100)) + \
            'NAME:               {}'.format(self.name.rstrip(b'\x00').decode())


class Modification(object):
    FORMAT = '<xx10H'

    def __init__(self, data, block_id=100):
        self.block_id = block_id
        self.service_codes = list(struct.unpack(Modification.FORMAT, data))

    def __repr__(self):
        return 'ID:                 MODIFICATION_{}\n'.format(self.block_id) + \
            ('SERVICECODE:        {}\n' * 10).format(*self.service_codes)


class Service(object):
    FORMAT = '<xx6sHIHH'

    def __init__(self, data, block_id=200):
        self.block_id = block_id
        self.department, self.service_date, self.ophours, self.repair_number, \
            self.service_code = struct.unpack(Service.FORMAT, data)

    def __repr__(self):
        return 'ID:                 SERVICE_{}\n'.format(self.block_id) + \
            'DEPARTMENT:         {}\n'.format(self.department.rstrip(b'\x00').decode()) + \
            'SERVICETDATE:       {}-{:02}-{:02}\n'.format(int(self.service_date / 512) + 1980,
                                                          int(int(self.service_date % 512) / 32),
                                                          int(int(self.service_date % 512) % 32)) + \
            'OPHOURS:            {}\n'.format(self.ophours) + \
            'REPAIRNUMBER:       {}\n'.format(self.repair_number) + \
            'SERVICECODE:        {}\n'.format(self.service_code)


class Statistics(object):
    FORMAT = '<xxI'

    def __init__(self, data, block_id=300):
        self.block_id = block_id
        self.ophours, = struct.unpack(Statistics.FORMAT, data)

    def __repr__(self):
        return 'ID:                 STATISTICS_{}\n'.format(self.block_id) + \
            'OPHOURS:            {}\n'.format(self.ophours)


class User(object):
    FORMAT = '<H'

    class Table(object):
        FORMAT = '<HHBBH3H'

        # TODO field three can be move to four, which also allows no further case switching during parsing
        DATATYPE_TO_STR = {1: ('0', None, None, lambda x, count: x),
                           2: ('UINT8', '{:3u}', numpy.uint8, lambda x, count: x),
                           3: ('UINT16', '{:5u}', numpy.uint16, lambda x, count: x),
                           4: ('UINT32', '{:10lu}', numpy.uint32, lambda x, count: x),
                           6: ('INT8', '{:4d}', numpy.int8, lambda x, count: x),
                           7: ('INT16', '{:5d}', numpy.int16, lambda x, count: x),
                           8: ('INT32', '{:10ld}', numpy.int32, lambda x, count: x),
                           9: ('DOUBLE', '{:f}', numpy.float64, lambda x, count: x),
                           10: ('FIX16D1', '{:7.1f}', numpy.int16, lambda x, count: x / 10),
                           11: ('FIX16D2', '{:7.2f}', numpy.int16, lambda x, count: x / 100),
                           12: ('FIX16D3', '{:7.3f}', numpy.int16, lambda x, count: x / 1000),
                           13: ('FIX32D1', '{:10.1f}', numpy.int32, lambda x, count: x / 10),
                           14: ('FIX32D2', '{:10.2f}', numpy.int32, lambda x, count: x / 100),
                           15: ('FIX32D3', '{:10.3f}', numpy.int32, lambda x, count: x / 1000),
                           23: ('RXTX_RX_GAIN_DESCRIPTOR', '', 8, lambda x, count: x),
                           24: ('EXT_RX_GAIN_DESCRIPTOR', '', 4, lambda x, count: x),
                           25: ('EXT_TX_GAIN_DESCRIPTOR', '', 4, lambda x, count: x),
                           26: ('FE_RX_GAIN_DESCRIPTOR', '', 4,
                                lambda x, count: numpy.frombuffer(x, [('ID', '<u1'), ('NomGain', '<i1'), ('RefGain', '<i1'), ('PowDPathAmpTxCon', '<u1')], count)),
                           27: ('FE_TX_GAIN_DESCRIPTOR', '', 4,
                                lambda x, count: numpy.frombuffer(x, [('ID', '<u1'), ('NomGain', '<i1'), ('RefGain', '<i1'), ('PowDAttPathRxCon', '<u1')], count)),
                           28: ('RX_GAIN_DESCRIPTOR', '', 10, lambda x, count: x),
                           29: ('TX_GAIN_DESCRIPTOR', '', 10, lambda x, count: x),
                           30: ('RXTX_FREQUENCY_BAND_BOUND', '', 8, lambda x, count: x),
                           31: ('RXTX_GAIN_SWITCH_BOUND', '', 8, lambda x, count: x),
                           32: ('GAIN_HARDWARE', '', 10, lambda x, count: x),
                           38: ('DIAG_TESTDESCRIPTOR', '', 8, lambda x, count: x),
                           39: ('PASSBAND', '', 8, lambda x, count: x),
                           40: ('UFIX16D4', '{:7.4f}', numpy.uint16, lambda x, count: x / 10000),
                           41: ('FE_POWERMEAS_SETTING_DESCRIPTOR', '', 10,
                                lambda x, count: numpy.frombuffer(x, [('ID', '<u1'), ('Inp', '<u1'), ('TimeC', '<u1'), ('TxCon', '<u1'), ('Ref-Freq', '<i4'), ('Ref-Pow', '<i2')], count)),
                           42: ('RXTX_DIAG_SETTINGS', '', 12, lambda x, count: x),
                           43: ('DIAG_DESCRIPTOR', '', 8,
                                lambda x, count: numpy.frombuffer(x, [('ID', '<u1'), ('act', '<u1'), ('min. (V)', '<i2'), ('max. (V)', '<i2'), ('FMDV (V)', '<i2')], count)),
                           44: ('REF_DIAG_SETTINGS', '', 13,
                                lambda x, count: numpy.frombuffer(x, [('ID', '<u1'), ('D6', '<u1'), ('NcfM', '<u1'), ('NcfD', '<u1'), ('ncf_freq', '<u4'), ('RfgI', '<u1'), ('rfg_freq', '<u4')], count)),
                           45: ('HW_CODE_DESCRIPTOR', '', 7, lambda x, count: x),
                           46: ('CHAR', '', numpy.char, lambda x, count: x),
                           47: ('GAIN_INDEX', '', 4, lambda x, count: x),
                           48: ('RX_LO3_SETTINGS', '', 18, lambda x, count: x),
                           49: ('LO3_SETTINGS', '', 18, lambda x, count: x),
                           50: ('IQIF_FILTER', '', 6, lambda x, count: x),
                           51: ('IQIF_CONTROL_SETTINGS', '', 10, lambda x, count: x),
                           52: ('IQIF_CONTROL_STRUCT', '', 10, lambda x, count: x),
                           53: ('IQIF_DIAG_SETTINGS', '', 10, lambda x, count: x),
                           54: ('IQIF_APPLICATION', '', 10, lambda x, count: x),
                           55: ('RXTX_RX_GAIN_DESCRIPTOR_2', '', 8, lambda x, count: x),
                           56: ('COMB_SETTINGS', '', 8, lambda x, count: x)}

        def __init__(self, data):
            self.rows, self.columns, self.row_id_type, self.col_id_type, \
                self.datatype, self.aux1, self.aux2, self.aux3 \
                = struct.unpack(User.Table.FORMAT, data[:struct.calcsize(User.Table.FORMAT)])
            start_byte = struct.calcsize(User.Table.FORMAT)
            if self.row_id_type != 1:
                row_id_info = User.Table.DATATYPE_TO_STR[self.row_id_type]
                byte_length = self.rows * row_id_info[2](0).nbytes
                row_id = numpy.frombuffer(data[start_byte:start_byte + byte_length], row_id_info[2])
                self.row_id = row_id_info[3](row_id, self.rows)
                start_byte += byte_length
                if start_byte % 2 == 1:
                    start_byte += 1
            else:
                self.row_id = None
            if self.col_id_type != 1:
                col_id_info = User.Table.DATATYPE_TO_STR[self.col_id_type]
                byte_length = self.columns * col_id_info[2](0).nbytes
                col_id = numpy.frombuffer(data[start_byte:start_byte + byte_length], col_id_info[2])
                self.col_id = col_id_info[3](col_id, self.columns)
                start_byte += byte_length
                if start_byte % 2 == 1:
                    start_byte += 1
            else:
                self.col_id = None
            datatype_info = User.Table.DATATYPE_TO_STR[self.datatype]
            try:
                element_size = datatype_info[2](0).nbytes
                byte_length = self.columns * self.rows * element_size
                data = numpy.frombuffer(data[start_byte:start_byte+byte_length], datatype_info[2])
            except:
                element_size = datatype_info[2]
                byte_length = self.rows * element_size
                data = data[start_byte:start_byte+byte_length]

            self.data = datatype_info[3](data, self.columns * self.rows).reshape(self.rows, self.columns)
            self.nbytes = start_byte + byte_length

        def __repr__(self):
            return 'ROWS:               {}\n'.format(self.rows) + \
                'COLUMNS:            {}\n'.format(self.columns) + \
                'ROW_ID_TYPE:        {}\n'.format(User.Table.DATATYPE_TO_STR[self.row_id_type][0]) + \
                'COLUMN_ID_TYPE:     {}\n'.format(User.Table.DATATYPE_TO_STR[self.col_id_type][0]) + \
                'DATATYPE:           {}\n'.format(User.Table.DATATYPE_TO_STR[self.datatype][0]) + \
                'AUX1:               {}\n'.format(self.aux1) + \
                'AUX2:               {}\n'.format(self.aux2) + \
                'AUX3:               {}\n'.format(self.aux3) + \
                str(self.data)


    def __init__(self, data, block_id=10000):
        self.block_id = block_id
        self.update_index, = struct.unpack(User.FORMAT, data[:struct.calcsize(User.FORMAT)])
        start_byte = struct.calcsize(User.FORMAT)
        self.tables = []
        while start_byte < len(data) - 1:
            table = User.Table(data[start_byte:])
            start_byte += table.nbytes
            self.tables.append(table)

    def __repr__(self):
        return 'ID:                 USER_{}\n'.format(self.block_id) + \
            'UPDATE_INDEX:       {:02}.{:02}\n'.format(int(self.update_index / 100), int(self.update_index % 100)) + \
            '\n'.join([str(tb) for tb in self.tables])

class End(object):
    def __init__(self, data, block_id=0xFFFF):
        self.block_id = block_id

    def __repr__(self):
        return 'ID:                 END\n'


with open('ref.bin', 'rb') as f:
    bin_data = f.read()

#with open('rxtx_0.bin', 'rb') as f:
#    bin_data = f.read()

#with open('fe.bin', 'rb') as f:
#    bin_data = f.read()

blocks = Block.from_binary(bin_data)
print('\n\n\n'.join([str(block) for block in blocks]))

