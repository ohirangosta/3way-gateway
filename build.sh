#! /bin/sh

COMP_OPSION=""

if [ $# = 1 ]; then
	if [ "$1" = "release" ]; then
		COMP_OPSION=""
	elif [ "$1" = "debug" ]; then
		COMP_OPSION="-DDEBUG"
	else
		echo "build error"
		echo "Usage: $0 < release | debug >"
	fi
else
	echo "build error"
	echo "Usage: $0 < release | debug >"
fi

if [ "$1" = "release" ]; then
	make clean
	rm ./can3way-transfer
	make
	gcc can3way-transfer.c parser.c lexer.o -o can3way-transfer -ll
elif [ "$1" = "debug" ]; then
	make clean
	rm ./can3way-transfer
	make
	gcc can3way-transfer.c parser.c lexer.o -o can3way-transfer.dbg -ll "$COMP_OPSION"
fi
