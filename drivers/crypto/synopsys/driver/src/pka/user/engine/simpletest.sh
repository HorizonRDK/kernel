#!/bin/sh

: ${SPEEDTESTS=rsa512 rsa1024 rsa2048}

echo "Running SW test."
openssl speed ${SPEEDTESTS}

echo "Running ENGINE test."
export OPENSSL_CONF=$PWD/simpletest.cnf
openssl speed -elapsed ${SPEEDTESTS}
