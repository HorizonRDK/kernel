#!/usr/bin/python2
import json
import types
import collections
import os
import binascii
import numpy as np 
import sys
import math

from array import array
from collections import OrderedDict  

def resolveJson(path):
    file = open(path, "rb")
    fileJson = json.load(file, object_pairs_hook=OrderedDict) 
    tmp_list = list(fileJson.values())

    return tmp_list

def resolveJsonKey(path):
    file = open(path, "rb")
    fileJson = json.load(file, object_pairs_hook=OrderedDict)
    tmp_list = list(fileJson.keys())

    return tmp_list

def bootInfoOutput():
    result = resolveJson(bootInfoPath)
    i = 0
    for i in range(len(result)):
        result[i] = int(result[i], 16)

    result = np.asarray(result, dtype=np.int32)
    return result


def bootloaderInfoOutput():
    result = resolveJson(bootLoaderPath)
    return result

def getFileSize(filePath):
    fsize = os.path.getsize(filePath)
    return fsize

def addCheckSum(filename):
    file_object = open(filename, 'rb')
    file_content = file_object.read()
    file_object.close()
 
    sum = 0
    for c in file_content:
        sum += ord(c) 
    return sum

def dtbNameTranfer(bootInfoContent, filename, num):
    listname = list(filename)

    i = 0
    j = num
    tmp = 0
    for i in range(len(listname)):
        tmp = tmp + (ord(listname[i])<<(8*(i%4)))
        if (i%4) == 3:
            bootInfoContent[j] = np.asarray(tmp , dtype=np.int32)
            j = j + 1
            tmp = 0
        if i+1 == len(listname):
            bootInfoContent[j] = np.asarray(tmp , dtype=np.int32)
            j = j + 1
            tmp = 0
    return tmp

def str2hex(s):
    odata = 0;
    su =s.upper()
    for c in su:
        tmp=ord(c)
        if tmp <= ord('9') :
            odata = odata << 4
            odata += tmp - ord('0')
        elif ord('A') <= tmp <= ord('F'):
            odata = odata << 4
            odata += tmp - ord('A') + 10
    return odata

bootInfoPath   = r"bootinfo.json"
bootLoaderPath = r"bootfile.json"
bootDtb     = r"bootdtb.json"


if __name__ == '__main__':

    if len(sys.argv) == 3:
        print 'len = 3'
        bootInfoPath   = sys.argv[1]
        bootLoaderPath = sys.argv[2]

    if len(sys.argv) == 2:
        bootInfoPath = sys.argv[1]
        print 'len = 2'

    bootInfoContent = bootInfoOutput()
    filePath = bootloaderInfoOutput()

    file = open(bootLoaderPath, "rb")
    fjson = json.load(file, object_pairs_hook=OrderedDict)

    bootInfoContent[0] = np.asarray(str2hex(fjson['imageaddr']), dtype=np.int32) 
    bootInfoContent[1] = np.asarray(str2hex(fjson['imagesize']), dtype=np.int32)
    bootInfoContent[2] = np.asarray(str2hex(fjson['recoveryaddr']), dtype=np.int32)
    bootInfoContent[3] = np.asarray(str2hex(fjson['recoverysize']), dtype=np.int32)

    file = open(bootDtb, "rb")
    hjson = json.load(file, object_pairs_hook=OrderedDict)

    j=5
    board_id = resolveJsonKey(bootDtb)
    bootInfoContent[4] = np.asarray(len(board_id), dtype=np.int32)
    for key in board_id:
        bootInfoContent[j] = np.asarray(str2hex(key), dtype=np.int32)
        bootInfoContent[j+1] = np.asarray(str2hex(hjson[key]['gpio_id']), dtype=np.int32)
        bootInfoContent[j+2] = np.asarray(str2hex(hjson[key]['dtb_addr']), dtype=np.int32)
        bootInfoContent[j+3] = np.asarray(str2hex(hjson[key]['dtb_size']), dtype=np.int32)
        dtbNameTranfer(bootInfoContent, hjson[key]['dtb_name'], j+4)
        j = j+12
    
    dtbname = resolveJson(bootLoaderPath)
    listname = list(dtbname[0])

    file_produced0  = open(filePath[5], 'wb')
    file_produced0.write(bootInfoContent)

    file_produced0.close()
