/*
 * can3-waytransfer.c
 *
 * Copyright (c) 2002-2009 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * reModeled by Oohira.
 * This program is transfer can packets from a can socket to from other can sockets. 
 * To analyze electric controller unit.
 *
 * usage:
 * 1)  ./can3-waytransfer 				: can packet(s) are not transfered
 * 2)  ./can3-waytransfer config.conf 	: can packet(s) are filtred and transfered engaged in config file
 *
 * how to compile:
 * ex) gcc cant3-wayransfer.c parser.c lexer.o -o cant3-wayransfer
 *
 * About ConfigFile:

   default_interface can0 pass can1 can2
   default_interface can1 pass can0 can2
   default_interface can2 pass can0 can1
   interface can0 canid 001-010 drop can1 can2
   interface can1 canid 0B4 drop can0

 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <netinet/in.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "terminal.h"
#include "lib.h"
#include "lib.c"

//for 3-way gateway
#include "token.h"

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define SILENT_INI 42 /* detect user setting on commandline */
#define SILENT_OFF 0  /* no silent mode */
#define SILENT_ANI 1  /* silent mode with animation */
#define SILENT_ON  2  /* silent mode (completely silent) */

#define BOLD    ATTBOLD
#define RED     ATTBOLD FGRED
#define GREEN   ATTBOLD FGGREEN
#define YELLOW  ATTBOLD FGYELLOW
#define BLUE    ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN    ATTBOLD FGCYAN

const char col_on [MAXCOL][19] = {BLUE, RED, GREEN, BOLD, MAGENTA, CYAN};
const char col_off [] = ATTRESET;

static char *cmdlinename[MAXSOCK];
static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 

#define MAXANI 4
const char anichar[MAXANI] = {'|', '/', '-', '\\'};

extern int optind, opterr, optopt;

static volatile int running = 1;

/* for filtering-rule */ 
#define NUMBEROF_MAXIMAMCANID 100
#define NUMBER_of_canfilterELEMENT 6 /* number of can_filter's elements */
struct can_filter_rule{ /* rules (default:: pass:0 period:0 start:-1 end:-1 diff:0)*/
		__u32 can_id; /* apply to CAN ID */
		__u32 can_id_range_start; /* apply to CAN ID RANGE START*/
		__u32 can_id_range_end; /* apply to CAN ID RANGE END*/
		int   pass_drop;   /* pass:1 or Drop(filtered):0 */
		int   period; /* use "period" for machine learning (0:don't use, 1:use) */
		int   start;  /* specify "start bit" for machine learning (-1:don't use, others:use) */
		int   end;	  /* specify "end bit" for machine learning (-1:don't use, others:use) */
		int   diff;   /* use "diff(difference)"" for machine learning (0:don't use, 1:use) */ 
		int   in_interface;
		int   out_interface1;
		int   out_interface2;
};

void sigterm(int signo)
{
	running = 0;
}

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}

// 3-way add function
#define I_CAN0 0 //I of Interface
#define I_CAN1 1
#define I_CAN2 2

//maximam can_id 000~7FF
#define CANID_MAXIMAMNUM 2048

int interface_char2int(char *can_interface) {
	if (strcmp(can_interface, "can0")==0) {
		return I_CAN0;
	} else if (strcmp(can_interface, "can1")==0) {
		return I_CAN1;	
	} else if (strcmp(can_interface, "can2")==0) {
		return I_CAN2;
	}
	return 0;
}

/* 3-way syntax tree merge struct can_filter_rule */
int merge_parser2can_filter_rule(struct AbstSyntaxTree *syntax_rule, struct can_filter_rule p[]) {
	//because i is rule_counter, return i+1
	int i = 0;
	while(syntax_rule != NULL) {
		p[i].pass_drop 		= syntax_rule->PassOrDrop;
		p[i].can_id 		= strtol(syntax_rule->can_id, (char **)NULL, 16);
		p[i].can_id_range_start = strtol(syntax_rule->can_id_range_start, (char **)NULL, 16);
		p[i].can_id_range_end	= strtol(syntax_rule->can_id_range_end, (char **)NULL, 16);
		p[i].in_interface 	= interface_char2int(syntax_rule->apply_rule_interface);
		p[i].out_interface1 	= syntax_rule->another_interface1 != "" ? interface_char2int(syntax_rule->another_interface1) : 0;
		p[i].out_interface2 	= syntax_rule->another_interface2 != "" ? interface_char2int(syntax_rule->another_interface2) : 0;
		i++;
		syntax_rule = syntax_rule->next_rule;
	}
	return i;
}

void Decimal2Hexadecimal(__u32 Dec, char *c) {
	unsigned char *str_ptr = &c[3];
	c[0] = '0';
	while(Dec) {
		*--str_ptr = Dec%16;
		if (10 > *str_ptr) {
			*str_ptr+=48;
		} else {
			*str_ptr+=55;
		}
		Dec/=16;
	}
}

int write_eth (int socket, struct sockaddr_in addr, struct can_frame *frame) {
	char send_data[100];
	sprint_canframe(send_data, frame, 0);
	if (sendto(socket, send_data, strlen(send_data), 0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("sendto_eth");
		return -1;
	}
	return 0;
}

extern FILE *yyin;

int main(int argc, char **argv)
{
	fd_set rdfs;
	int s[MAXSOCK];
	int bridge = 0;				/*can raw wscoket*/
	useconds_t bridge_delay = 0;
	unsigned char timestamp = 0;
	unsigned char dropmonitor = 0;
	unsigned char silent = SILENT_INI;
	unsigned char silentani = 0;
	unsigned char color = 0;
	unsigned char view = 0;
	unsigned char log = 0;
	unsigned char logfrmt = 0;
	int count = 0;
	int rcvbuf_size = 0;
	int opt, ret;
	int currmax, numfilter;
	char *ptr, *nptr;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct can_filter *rfilter;
	can_err_mask_t err_mask;
	struct can_frame frame;
	int nbytes, i;
	struct ifreq ifr;
	struct timeval tv, last_tv;
	FILE *logfile = NULL;

	//add for 3way-gateway
	char *configfilename;
	struct ifreq ifr2;
	struct sockaddr_can addr2;
	int bridge_re = 0;
	struct iovec iov2; //for dump
	struct msghdr msg2; //for dump
	struct can_frame frame2;
	int IDcan0 = 0;
	int IDcan1 = 0;
	int IDcan2 = 0;
	char INcan0_Interface_table[2][CANID_MAXIMAMNUM];
	char INcan1_Interface_table[2][CANID_MAXIMAMNUM];
	char INcan2_Interface_table[2][CANID_MAXIMAMNUM];
	int can0_masked = 1;
	int can1_masked = 1;
	int can2_masked = 1;
	int j;

	//for filtering-rule
	int apply_rule = 0;
	int i_check;
	int rule_counter = 0;
	struct can_filter_rule canid_filter[NUMBEROF_MAXIMAMCANID]; /*for filtering*/

    //initialize
	for(i_check=0;i_check<NUMBEROF_MAXIMAMCANID;i_check++){
			canid_filter[i_check].can_id 			= 2048; 
			canid_filter[i_check].can_id_range_start 	= 2048; 
			canid_filter[i_check].can_id_range_end 		= 2048; 
			canid_filter[i_check].pass_drop  		= 0; 
			canid_filter[i_check].period 			= 0; 
			canid_filter[i_check].start  			= -1;
			canid_filter[i_check].end    			= -1;
			canid_filter[i_check].diff   			= 0;
			canid_filter[i_check].in_interface  	= 0;
			canid_filter[i_check].out_interface1  	= 0;
			canid_filter[i_check].out_interface2 	= 0;
	}

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	last_tv.tv_sec  = 0;
	last_tv.tv_usec = 0;

	bridge = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	bridge_re = socket(PF_CAN, SOCK_RAW, CAN_RAW); /* for retransfer */
	if (bridge < 0) {
		perror("bridge socket");
		return 1;
	}
	addr.can_family = AF_CAN;
	addr2.can_family = AF_CAN;
	strcpy(ifr.ifr_name, "can1");//can1
	strcpy(ifr2.ifr_name, "can0");//can0 /*copy for tansfer*/

	if (ioctl(bridge, SIOCGIFINDEX, &ifr) < 0)
		perror("SIOCGIFINDEX");

	addr.can_ifindex = ifr.ifr_ifindex;
	addr2.can_ifindex = ifr2.ifr_ifindex; /*add family for transfer*/

	if (!addr.can_ifindex) {
		perror("invalid bridge interface");
		return 1;
	}

	/* disable default receive filter on this write-only RAW socket */
	setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	const int loopback = 0;
	setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
	
	if (bind(bridge, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bridge bind1");
		return 1;
	}
			
	/* load a config file */
	if (argc == 2) apply_rule = 1;
	yyin = fopen(argv[1], "r");
	init_buf();
	struct AbstSyntaxTree *syntax_rule = parse();
	rule_counter = merge_parser2can_filter_rule(syntax_rule, canid_filter);
#ifdef DEBUG
	printf("syntax OK!!!!!!!!\n");
	print_rule(syntax_rule);
	printf("[CONFIG] CANID:%x PASS:%d IN_INTERFACE:%d OUT1:%d OUT2:%d\n",canid_filter[0].can_id, canid_filter[0].pass_drop, canid_filter[0].in_interface, canid_filter[0].out_interface1, canid_filter[0].out_interface2);
#endif

	currmax = 2; /* find real number of CAN devices */

	/* UDP sender for can2 */
	struct sockaddr_in addr_eth;
	if ((s[2] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("eth socket");
		return 1;
	}
	/* bind IDcan2 and s[i] */
	IDcan2 = s[2];

	addr_eth.sin_family = AF_INET;
	addr_eth.sin_port = htons(4989);
	addr_eth.sin_addr.s_addr = inet_addr("169.254.19.16");

	for (i=0; i < currmax; i++) {

		if (i == 0) 	ptr = "can0";
		else		ptr = "can1";
		nptr = strchr(ptr, ',');

#ifdef DEBUG
		printf("open %d '%s'.\n", i, ptr);
#endif

		// s[0]:can socket s[1]:can socket s[2]:ethernet socket
		/* Create CAN socket in s[i] */
		s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW); //regist to bind
		if (s[i] < 0) {
			perror("socket");
			return 1;
		}

		/* bind canID and s[i] */
		if(!IDcan0) IDcan0 = s[i];
		else if(!IDcan1) IDcan1 = s[i];

		cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

		if (nptr)
			nbytes = nptr - ptr;  /* interface name is up the first ',' */
		else
			nbytes = strlen(ptr); /* no ',' found => no filter definitions */

		if (nbytes >= IFNAMSIZ) {
			fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
			return 1;
		}

		if (nbytes > max_devname_len)
			max_devname_len = nbytes; /* for nice printing */

		addr.can_family = AF_CAN;	/* for dump? */

		memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
		strncpy(ifr.ifr_name, ptr, nbytes);

		memset(&ifr2.ifr_name, 0, sizeof(ifr2.ifr_name)); /* for transfer */
		strncpy(ifr2.ifr_name, ptr, nbytes); 		  /* for transfer*/

#ifdef DEBUG
		printf("using interface name '%s'.\n", ifr.ifr_name);
#endif
		if (strcmp(ANYDEV, ifr.ifr_name)) {
			if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) {
				perror("SIOCGIFINDEX");
				exit(1);
			}
			addr.can_ifindex = ifr.ifr_ifindex;
		} else
			addr.can_ifindex = 0; /* any can interface */

		if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}
	}


	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	//table initial
	for (i = 0; i < 2; i++) {
		for (j = 0; j < CANID_MAXIMAMNUM; j++) {
			INcan0_Interface_table[i][j] = 255;
			INcan1_Interface_table[i][j] = 255;
			INcan2_Interface_table[i][j] = 255;
		}
	}
	/*
	 * INcan*_Interface_table[2][2048] example : can0 only dump and send can1, can2
	 *	  
         *    INcan0_interfece_table
	 *
	 *       Sendcan1 Sendcan2	
	 *CAN ID|--------|--------|
	 *   000|1 (pass)|1 (pass)|
	 *   001|1 (pass)|1 (pass)|
	 *   002|1 (pass)|1 (pass)|
	 *   003|1 (pass)|1 (pass)|
	 *	| ・・・ | ・・・ |	
	 *   7FD|1 (pass)|1 (pass)|
	 *   7FE|1 (pass)|1 (pass)|
	 *   7FF|1 (pass)|1 (pass)|
	 *
         *    INcan1_interfece_table
	 *
	 *       Sendcan0 Sendcan2	
	 *CAN ID|--------|--------|
	 *   000|0 (drop)|0 (drop)|
	 *   001|0 (drop)|0 (drop)|
	 *   002|0 (drop)|0 (drop)|
	 *   003|0 (drop)|0 (drop)|
	 *	| ・・・ | ・・・ |	
	 *   7FD|0 (drop)|0 (drop)|
	 *   7FE|0 (drop)|0 (drop)|
	 *   7FF|0 (drop)|0 (drop)|
	 *
         *    INcan2_interfece_table
	 *
	 *       Sendcan0 Sendcan1	
	 *CAN ID|--------|--------|
	 *   000|0 (drop)|0 (drop)|
	 *   001|0 (drop)|0 (drop)|
	 *   002|0 (drop)|0 (drop)|
	 *   003|0 (drop)|0 (drop)|
	 *	| ・・・ | ・・・ |	
	 *   7FD|0 (drop)|0 (drop)|
	 *   7FE|0 (drop)|0 (drop)|
	 *   7FF|0 (drop)|0 (drop)|
	 *
	 */

	//interface filter rule create
	for(i_check=0;i_check<rule_counter;i_check++) {
		if (canid_filter[i_check].in_interface == I_CAN0) {
			if (canid_filter[i_check].can_id == 2048) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan0_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan0_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			} else {
				//id rule
#ifdef DEBUG
				printf("[can0 can_id_rule] %d - %d %d\n", canid_filter[i_check].can_id_range_start, canid_filter[i_check].can_id_range_end, canid_filter[i_check].can_id);
#endif
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) if (canid_filter[i_check].can_id_range_start <= j && j <= canid_filter[i_check].can_id_range_end || j == canid_filter[i_check].can_id) INcan0_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) if (canid_filter[i_check].can_id_range_start <= j && j <= canid_filter[i_check].can_id_range_end || j == canid_filter[i_check].can_id) INcan0_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			}
		} else if (canid_filter[i_check].in_interface == I_CAN1) {
			if (canid_filter[i_check].can_id == 2048) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan1_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan1_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			} else {
				//id rule
#ifdef DEBUG
				printf("[can1 can_id_rule] %d - %d %d\n", canid_filter[i_check].can_id_range_start, canid_filter[i_check].can_id_range_end, canid_filter[i_check].can_id);
#endif
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) for (j = 0; j < CANID_MAXIMAMNUM; j++) if (canid_filter[i_check].can_id_range_start <= j && j <= canid_filter[i_check].can_id_range_end || j == canid_filter[i_check].can_id) INcan1_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) if (canid_filter[i_check].can_id_range_start <= j && j <= canid_filter[i_check].can_id_range_end || j == canid_filter[i_check].can_id) INcan1_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			}
		} else if (canid_filter[i_check].in_interface == I_CAN2) {
			if (canid_filter[i_check].can_id == 2048) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan2_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan2_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			} else {
				//id rule
#ifdef DEBUG
				printf("[can2 can_id_rule] %d - %d %d\n", canid_filter[i_check].can_id_range_start, canid_filter[i_check].can_id_range_end, canid_filter[i_check].can_id);
#endif
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) for (j = 0; j < CANID_MAXIMAMNUM; j++) if (canid_filter[i_check].can_id_range_start <= j && j <= canid_filter[i_check].can_id_range_end || j == canid_filter[i_check].can_id) INcan2_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) if (canid_filter[i_check].can_id_range_start <= j && j <= canid_filter[i_check].can_id_range_end || j == canid_filter[i_check].can_id) INcan2_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			}	
		}
	}

	for (i = 0; i < 2; i++) {
		for (j = 0; j < CANID_MAXIMAMNUM; j++) {
			if (INcan0_Interface_table[i][j] == 255) INcan0_Interface_table[i][j] = 0;
			if (INcan1_Interface_table[i][j] == 255) INcan1_Interface_table[i][j] = 0;
			if (INcan2_Interface_table[i][j] == 255) INcan2_Interface_table[i][j] = 0;
		}
	}

#ifdef DEBUG
	/* table print */
	for (j = 0; j < CANID_MAXIMAMNUM; j++) {
		printf("%d|%d|%d|\t%d|%d|%d|\t%d|%d|%d|\n", j, 
			INcan0_Interface_table[0][j], INcan0_Interface_table[1][j], j,
			INcan1_Interface_table[0][j], INcan1_Interface_table[1][j], j,
			INcan2_Interface_table[0][j], INcan2_Interface_table[1][j]);
	}
#endif

	//dump main
	while (running) {

		can0_masked = 1;
		can1_masked = 1;
		can2_masked = 1;

		FD_ZERO(&rdfs);
		for (i=0; i<currmax + 1; i++)
			FD_SET(s[i], &rdfs);

		if ((ret = select(s[currmax-1]+1, &rdfs, NULL, NULL, NULL)) < 0) {
			//perror("select");
			running = 0;
			continue;
		}

		for (i=0; i<currmax + 1; i++) {  /* check all CAN and eth RAW sockets */

			if (FD_ISSET(s[i], &rdfs)) {

				int idx;

				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;

				if (i >= currmax) {
					//can2 recv

				} else {
					//can0, can1 recv
					nbytes = recvmsg(s[i], &msg, 0); /* message receiving from s[i] <- target */
					if (nbytes < 0) {
						perror("read");
						return 1;
					}
				}

				if (nbytes < sizeof(struct can_frame)) {
					fprintf(stderr, "read: incomplete CAN frame\n");
					return 1;
				}

#ifdef DEBUG
				printf("(DEBUG - FRAME CHECK) ID:%x , DLC:%d, DATA[0]:%x [1]:%x [2]:%x [3]:%x [4]:%x [5]:%x [6]:%x [7]:%x\n",frame.can_id,frame.can_dlc,frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
#endif
				//mask can_id
				//send
				//mask = 0:bridge, 1:filter
				if(s[i] == IDcan0) {
						//mask can_id
						if(apply_rule) {
								if (INcan0_Interface_table[0][frame.can_id]) can1_masked = 0;
								if (INcan0_Interface_table[1][frame.can_id]) can2_masked = 0;
						} else {
							can1_masked = 0;
							can2_masked = 0;
						}

						if (!can1_masked) nbytes = write(IDcan1, &frame, sizeof(struct can_frame));
						if (!can2_masked) nbytes = write_eth(IDcan2, addr_eth, &frame);

				}
				if(s[i] == IDcan1) {
						//mask can_id
						if(apply_rule) {
								if (INcan1_Interface_table[0][frame.can_id]) can0_masked = 0;
								if (INcan1_Interface_table[1][frame.can_id]) can2_masked = 0;
						} else {
							can0_masked = 0;
							can2_masked = 0;
						}

						if (!can0_masked) nbytes = write(IDcan0, &frame, sizeof(struct can_frame));
						if (!can2_masked) nbytes = write_eth(IDcan2, addr_eth, &frame);

				}
				if(s[i] == IDcan2) {
						//mask can_id
						if(apply_rule) {
								if (INcan2_Interface_table[0][frame.can_id]) can0_masked = 0;
								if (INcan2_Interface_table[1][frame.can_id]) can1_masked = 0;
						} else {
							can0_masked = 0;
							can1_masked = 0;
						}

						if (!can0_masked) nbytes = write(IDcan0, &frame, sizeof(struct can_frame));
						if (!can2_masked) nbytes = write(IDcan1, &frame, sizeof(struct can_frame));
				}

				idx = idx2dindex(addr.can_ifindex, s[i]);
#ifdef DEBUG
				printf(" %s", (color>2)?col_on[idx%MAXCOL]:"");

				printf(" %s", (color && (color<3))?col_on[idx%MAXCOL]:"");
				printf("%*s", max_devname_len, devname[idx]);
				printf("%s  ", (color==1)?col_off:"");
				
				fprint_long_canframe(stdout, &frame, NULL, view); /* show can-frame */

				printf("%s", (color>1)?col_off:"");
				printf("\n");
#endif
			}

		out_fflush:
			fflush(stdout);
		}
	}// end of while(running)

	for (i=0; i<currmax; i++)
		close(s[i]);

	if (bridge){
		close(IDcan0);
		close(IDcan1);
	}


	return 0;
}
