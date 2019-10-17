#!/bin/bash
for f in elpsaspadiag.ko elpsaspa.ko elpmpmdiag.ko elpeadiag.ko elpmpm.ko elpkepdev.ko elpreusr.ko elpkep.ko elpre.ko elpea.ko elpclp800.ko elpclp890.ko elppka.ko elpspaccusr.ko elpspacccrypt.ko elpspacc.ko elpmem.ko elppdu.ko; do
   if [ -e $f ]; then rmmod $f; fi
done

