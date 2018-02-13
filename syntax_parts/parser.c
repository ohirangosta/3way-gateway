#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "token.h"

struct AbstSyntaxTree *parse();

int yylval;
int yylex();
FILE *yyin;

#define BUF_MAX 256

char buf[BUF_MAX];
char *yytext;

#define EMPTY -2

int saved_marker;

void init_buf() {
	saved_marker = EMPTY;
}

int scan() {
	int m = yylex();
	return m;
}

int nexttoken() {
	int m;
	if (saved_marker != EMPTY) {
		m = saved_marker;
	} else {
		m = scan();
		while (m == SPACE || m == NL) {
			m = scan();
		}
		saved_marker = m;
	}
	return m;
}

void syntax_error() {
	fprintf(stderr, "Syntax Error\n");
	exit(-1);
}

void eat_token() {
	saved_marker = EMPTY;
}

void parse_interface2(struct AbstSyntaxTree *rule) {
	int m = nexttoken();
	switch (m) {
		case CAN0:
			eat_token();
		        strcpy(rule->another_interface2, yytext);
			break;
		case CAN1:
			eat_token();
			strcpy(rule->another_interface2, yytext);
			break;
		case CAN2:
			eat_token();
		        strcpy(rule->another_interface2, yytext);
			break;
	}
}

void parse_interface1(struct AbstSyntaxTree *rule) {
	int m = nexttoken();
	switch (m) {
		case CAN0:
			eat_token();
			strcpy(rule->another_interface1, yytext);
			parse_interface2(rule);
			break;
		case CAN1:
			eat_token();
		        strcpy(rule->another_interface1, yytext);
			parse_interface2(rule);	
			break;
		case CAN2:
			eat_token();
		        strcpy(rule->another_interface1, yytext);
			parse_interface2(rule);	
			break;
		default:
			syntax_error();
	}
}

void parse_PassDrop(struct AbstSyntaxTree *rule) {
	int m = nexttoken();
	switch (m) {
	case PASS:
		eat_token();
		rule->PassOrDrop = 1;
		parse_interface1(rule);
		break;
	case DROP:
		eat_token();
		rule->PassOrDrop = 0;
		parse_interface1(rule);
		break;
	default:
		syntax_error();
	}
}

void parse_canid(struct AbstSyntaxTree *rule) {
	int m = nexttoken();
	if (m == CAN_ID) {
		eat_token();
		m = nexttoken();
		if (m == CAN_ID_NUM) {
			eat_token();
			strcpy(rule->can_id, yytext);
		} else {
			syntax_error();
		}
	} else {
		syntax_error();
	}
}

struct AbstSyntaxTree *parse() {
	struct AbstSyntaxTree *rule = malloc(sizeof(struct AbstSyntaxTree));
	int m = nexttoken();
	switch (m) {
		case DEFAULT_INTERFACE:
			eat_token();
			m = nexttoken();
			if (m == CAN0 || m == CAN1 || m == CAN2) {
					eat_token();
					strcpy(rule->apply_rule_interface, yytext);
					parse_PassDrop(rule);
			} else {	
				syntax_error();
			}
			break;
		case INTERFACE:
			eat_token();
			m = nexttoken();
			if (m == CAN0 || m == CAN1 || m == CAN2) {
					eat_token();
					strcpy(rule->apply_rule_interface, yytext);
					parse_canid(rule);
					parse_PassDrop(rule);
			} else {	
				syntax_error();
			}
			break;
		case ENDOFFILE:
			return rule;
			break;
		default:
			syntax_error();
	}
	
        m = nexttoken();
        if (m == INTERFACE || m == DEFAULT_INTERFACE) rule->next_rule = parse(); //recursive call
	return rule;
}

void print_rule(struct AbstSyntaxTree *rule) {
	printf("pass or drop:%d, applly:%s, another1:%s, another2:%s, CAN ID:%s\n", rule->PassOrDrop, rule->apply_rule_interface, rule->another_interface1, rule->another_interface2, rule->can_id);
	if (rule->next_rule != NULL) print_rule(rule->next_rule);
}