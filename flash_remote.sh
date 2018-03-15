#!/bin/sh

bin_file=$1
address=$2

scp $bin_file debugpi:

echo "program $bin_file $address" | nc debugpi 4444
echo "reset halt" | nc debugpi 4444
