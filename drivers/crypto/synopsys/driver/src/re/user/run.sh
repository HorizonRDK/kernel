#!/bin/bash
log=epn5604_881094F9_re.log
echo "cipher,hash,blocksize,wall time(sec),user time(sec),sys time (sec),CPU pct,mbit/sec,job/sec,cpu pct/job" > $log
for ciphers in aes128gcm aes256gcm; do
for sizes in 512 1024 2048 4096 8192 16384; do
    echo -n "$ciphers,NULL,$sizes," >> $log
    ./reuserspd --size $sizes --runs 10000 --cipher $ciphers --hash null | tee -a $log 
done;
done

for ciphers in null aes128cbc aes256cbc descbc 3descbc rc440 rc4128; do
for hashes in null md5 md580 sha1 sha180 sha256; do
for sizes in 512 1024 2048 4096 8192 16384; do
   echo -n "$ciphers,$hashes,$sizes," >> $log
    ./reuserspd --size $sizes --runs 10000 --cipher $ciphers --hash $hashes | tee -a $log
done;
done;
done;
