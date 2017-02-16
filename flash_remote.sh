#!/bin/sh

bin_file=$1
address=$2

scp $bin_file growpi:

echo "program $bin_file $address" | nc growpi 4444
echo "reset halt" | nc growpi 4444
