#! /bin/sh

make clean
rm ./can3way-transfer
make
gcc can3way-transfer.c parser.c lexer.o -o can3way-transfer -ll
