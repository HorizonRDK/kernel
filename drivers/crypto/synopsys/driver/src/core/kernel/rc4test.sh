#!/bin/bash
# This script simply loads and unloads the rc4 test module.
# Requires elpspaccusr module.
# See dmesg for results of test.
insmod spaccrc4test.ko
rmmod spaccrc4test
