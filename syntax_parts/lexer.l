%{
#include "token.h"
%}
%%
"default_interface"			{ return DEFAULT_INTERFACE; }
"interface"				{ return INTERFACE; }
"can0"					{ return CAN0; }
"can1"					{ return CAN1; }
"can2"					{ return CAN2; }
"pass"					{ return PASS; }
"drop"					{ return DROP; }
[0-9A-F][0-9A-F][0-9A-F]		{ return CAN_ID_NUM; }
"canid"					{ return CAN_ID; }
" "+					{ return SPACE; }
"\n"					{ return NL; }
.					{ return UD; }
