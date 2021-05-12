#!/usr/bin/env python
import struct
from zlib import crc32
import sys

def getCrc32(filename):
    with open(filename, 'rb') as f:
        return crc32(f.read()) & 0xffffffff

if len(sys.argv) < 2:
    print('You must enter the file')
    exit(1)
elif len(sys.argv) > 3:
    print('Only one file is permitted')
    exit(1)

filename = sys.argv[1]
#with open(sys.argv[2], "w") as fi:
#    fi.write(getCrc32(filename));
crc_ret = getCrc32(filename);
string_ret = "crc ret %x" %(crc_ret)
print(string_ret);
print('{:8} {:x}'.format('crc32:', getCrc32(filename)))

with open(sys.argv[2], 'wb')as fi:
    a = struct.pack('I', crc_ret)
    fi.write(a);
