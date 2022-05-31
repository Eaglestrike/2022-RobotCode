import struct

import numpy as np


TYPE_ENUM = {
    1: ('UINT8', 1, np.uint8, 'c'),
    2: ('INT32', 4, np.int32, 'i'),
    3: ('UINT32', 4, np.uint32, 'I'),
    4: ('FLOAT32', 4, np.float32, 'f'),
    5: ('FLOAT64', 8, np.float64, 'd'),
}


class DataLog:
    def __init__(self):
        self.time = []
        self.data = {}


def load_datalog(log_path):
    print(f'Loading datalog file {log_path}...', end='')
    result = DataLog()
    numpy_typemap = {}
    record_keys = []
    with open(log_path, 'rb') as fp:
        # Read a uint32 number of fields
        num_fields = struct.unpack('@I', fp.read(4))[0]
        print(f'Identified {num_fields} fields')
        if num_fields < 1:
            return None

        # Read the next num_fields chunks of the header
        decode_str = '=d'
        record_bytes = 8
        for field_id in range(num_fields):
            # Read the uint8 datatype enum
            datatype_enum = struct.unpack('@B', fp.read(1))[0]
            type_label, type_size, numpy_type, fmt_char = TYPE_ENUM[datatype_enum]
            decode_str += fmt_char
            record_bytes += type_size
            print(f'  {type_label} ', end='')

            # Extract the uint32 key length
            key_length = struct.unpack('@I', fp.read(4))[0]
            key_str = struct.unpack("%ds" % key_length, fp.read(key_length))[0].decode('ascii')
            print(f'"{key_str}" ({key_length})')

            # Initialize the corresponding python arrays
            result.data[key_str] = []
            numpy_typemap[key_str] = numpy_type
            record_keys.append(key_str)

        # Done reading header, now start loading the records
        print('Reading records...')
        print(f'Record length {record_bytes} bytes decoding with "{decode_str}"')
        record_buf = fp.read(record_bytes)
        while record_buf:
            unpacked_record = struct.unpack(decode_str, record_buf)

            # Append the timestamp first
            result.time.append(unpacked_record[0])

            # Then append the rest of the fields
            for key, value in zip(record_keys, unpacked_record[1:]):
                if numpy_typemap[key] == np.uint8:
                    conditioned_value = int.from_bytes(value, 'little')
                else:
                    conditioned_value = value
                result.data[key].append(conditioned_value)

            # Read the next record from the file
            record_buf = fp.read(record_bytes)

    print(f'Loaded {len(result.time)} records from datalog.')

    # Finally convert to numpy arrays
    result.time = np.array(result.time, dtype=np.float64)
    for key in result.data.keys():
        result.data[key] = np.array(result.data[key], dtype=numpy_typemap[key])

    return result


if __name__ == '__main__':
    logfile = load_datalog('/Users/sammy/data-logger/cmake-build-debug/test.log')
    print(logfile.time)
    print(logfile.data)

    import matplotlib.pyplot as plt
    plt.plot(logfile.time, logfile.data['float32_data'])
    plt.show()
