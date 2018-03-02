3way-gateway
====

###Overview

3way-gateway is vehicle network analysis tool. The tool can sniff messages sent by specific ECUs.

## Description

3way-gateway is vehicle network analysis tool. The tool can sniff messages sent by specific ECUs. For example, we can send packets sniffed packets with can0 with CAN IDs 000 to 099 to can 1 , and packets with CAN IDs 100 to 7 FF can be sent to can 2. Also, although it is necessary to physically block the CAN network, we can identify all the CAN IDs transmitted by the ECU of interest. By doing this, it is possible to impersonate perfectly if you are impersonating an identified CAN ID when the ECU you are interested in is bus off.

## Directory Structure

3way-gateway  
┣━ 3way-gateway-child  
┃	┣━ build_child.sh  
┃	┣━ can3way-gateway-child  
┃	┣━ lib.c  
┃	┗━ lib.h  
┣━ config_example  
┃	┣━ example1.conf  
┃	┣━ example2.conf  
┃	┣━ example2.conf  
┃	┗━ test.conf  
┣━ Makefile  
┣━ build.sh  
┣━ can3way-transfer.c  
┣━ lexer.l  
┣━ lib.c  
┣━ lib.h  
┣━ parse.c  
┣━ terminal.h  
┗━ token.h  

## Requirement

Hardware
	Raspberry Pi 3 model B * 2, PiCAN Duo, PiCAN
Software
	C, lex, CAN

## Usage

Please execute below on Raspberry Pi equipped "two" CAN interface.  
$ ./can3way-transfer [config file]  
Please execute below on Raspberry Pi equipped "one" CAN interface.  
$ ./can3way-gateway-child  

## Install

$ git clone https://github.com/ohirangosta/3way-gateway  
$ cd 3way-gateway  
$ ./build.sh release  

## Contribution

## Author

[rangosta](https://github.com/ohirangosta)
