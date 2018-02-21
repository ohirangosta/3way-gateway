
/*
 * can3-waytransfer.c
 *
 * Copyright (c) 2002-2009 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * reModeled by Oohira.
 * This program is transfer can packets from a can socket to from other can sockets. 
 * To combine filtering program with Machine learning.
 *
 * usage:
 * 1)  ./can3-waytransfer 				  : can packet(s) are not transfered
 * 2)  ./can3-waytransfer < config.conf -f: can packet(s) are filtred and transfered engaged in config file
 *
 * how to compile:
 * ex) gcc cant3-wayransfer.c parser.c lexer.c -lib.c -o cant3-wayransfer
 *
 * About ConfigFile:
# Speed value
id=020
pass=0
period=-1
start=-1
end=-1
#diff=4
diff=-1

# RPM
id=022
・・・

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
		int   pass;   /* pass:1 or Drop(filtered):0 */
		int   period; /* use "period" for machine learning (0:don't use, 1:use) */
		int   start;  /* specify "start bit" for machine learning (-1:don't use, others:use) */
		int   end;	  /* specify "end bit" for machine learning (-1:don't use, others:use) */
		int   diff;   /* use "diff(difference)"" for machine learning (0:don't use, 1:use) */ 
		int   in_interface;
		int   out_interface1;
		int   out_interface2;
};

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: [options]\n");
	fprintf(stderr, "  (use CTRL-C to terminate %s)\n\n", prg);
	fprintf(stderr, "         -f <configfile.conf>  (load a configure file to filter, machine-learn)\n");
	fprintf(stderr, "         -a          			(enable additional ASCII output)\n");
	fprintf(stderr, "         -s <level>  			(silent mode - %d: off (default) %d: animation %d: silent)\n", SILENT_OFF, SILENT_ANI, SILENT_ON);
	fprintf(stderr, "         -u <usecs>  			(delay bridge forwarding by <usecs> microseconds)\n");
	fprintf(stderr, "         -l         			(log CAN-frames into file. Sets '-s %d' by default)\n", SILENT_ON);
	fprintf(stderr, "         -n <count>  			(terminate after receiption of <count> CAN frames)\n");
	fprintf(stderr, "         -r <size>  			(set socket receive buffer to <size>)\n");
	fprintf(stderr, "         -d          			(monitor dropped CAN frames)\n");
	fprintf(stderr, "         -e         			(dump CAN error frames in human-readable format)\n");
	fprintf(stderr, "         -D         			(DEBUG-MODE ON)\n");

	fprintf(stderr, "\n");
	fprintf(stderr, "\nConfigure file Example:\n");
	fprintf(stderr, "Up to %d CAN interfaces with optional filter sets can be specified\n", MAXSOCK);
	fprintf(stderr, "on the commandline in the form: <ifname>[,filter]*\n");
	fprintf(stderr, "\nComma separated filters can be specified for each given CAN interface:\n");
	fprintf(stderr, " <can_id>:<can_mask> (matches when <received_can_id> & mask == can_id & mask)\n");
	fprintf(stderr, " <can_id>~<can_mask> (matches when <received_can_id> & mask != can_id & mask)\n");
	fprintf(stderr, " #<error_mask>       (set error frame filter, see include/linux/can/error.h)\n");
	fprintf(stderr, "\nCAN IDs, masks and data content are given and expected in hexadecimal values.\n");
	fprintf(stderr, "When can_id and can_mask are both 8 digits, they are assumed to be 29 bit EFF.\n");
	fprintf(stderr, "Without any given filter all data frames are received ('0:0' default filter).\n");
	//fprintf(stderr, "\nUse interface name '%s' to receive from all CAN interfaces.\n", ANYDEV);
	fprintf(stderr, "\nExamples:\n");
	fprintf(stderr, "%s -l any,0~0,#FFFFFFFF    (log only error frames but no(!) data frames)\n", prg);
	fprintf(stderr, "%s -l any,0:0,#FFFFFFFF    (log error frames and also all data frames)\n", prg);
	fprintf(stderr, "%s vcan2,92345678:DFFFFFFF (match only for extended CAN ID 12345678)\n", prg);
	fprintf(stderr, "%s vcan2,123:7FF (matches CAN ID 123 - including EFF and RTR frames)\n", prg);
	fprintf(stderr, "%s vcan2,123:C00007FF (matches CAN ID 123 - only SFF and non-RTR frames)\n", prg);
	fprintf(stderr, "\n");
}

void sigterm(int signo)
{
	running = 0;
}

// 3-way add function
#define I_CAN0 0 //I of Interface
#define I_CAN1 1
#define I_CAN2 2

//eth0 is number
#define ThreeWAYGW_eth 1
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
void merge_parser2can_filter_rule(struct AbstSyntaxTree *syntax_rule, struct can_filter_rule p[]) {
	int i = 0;
	while(syntax_rule != NULL) {
		p[i].pass 				= syntax_rule->PassOrDrop;
		p[i].can_id 			= strtol(syntax_rule->can_id, (char **)NULL, 16);
		p[i].in_interface 		= interface_char2int(syntax_rule->apply_rule_interface);
		p[i].out_interface1 	= syntax_rule->another_interface1 != "" ? interface_char2int(syntax_rule->another_interface1) : 0;
		p[i].out_interface2 	= syntax_rule->another_interface2 != "" ? interface_char2int(syntax_rule->another_interface2) : 0;
		i++;
		syntax_rule = syntax_rule->next_rule;
	}
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

	/*add for tansfer each other*/
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
	int DEBUG_MODE=0;

	//for filtering-rule
	int masked = 1;
	int apply_rule = 0;
	int i_check;
	struct can_filter_rule canid_filter[NUMBEROF_MAXIMAMCANID]; /*for filtering*/
    //initialize
	for(i_check=0;i_check<NUMBEROF_MAXIMAMCANID;i_check++){
			canid_filter[i_check].can_id 			= 0; 
			canid_filter[i_check].pass 		  		= 0; 
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
	strcpy(ifr.ifr_name, "can0");//can1
	strcpy(ifr2.ifr_name, "can1");//can0 /*copy for tansfer*/

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

	if (opt == 'B') {
		const int loopback = 0;

		setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
			   &loopback, sizeof(loopback));
	}

	if (bind(bridge, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bridge bind1");
		return 1;
	}
	

	apply_rule = 1;
	// 3-way add
	yyin = fopen(argv[1], "r");
	init_buf();
	struct AbstSyntaxTree *syntax_rule = parse();
	merge_parser2can_filter_rule(syntax_rule, canid_filter);
#ifdef DEBUG
	printf("syntax OK\n");
#endif

	currmax = 3; /* find real number of CAN devices */

	// UDP server for can2
	struct sockaddr_in addr_eth;
	if ((s[2] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("eth socket");
		return 1;
	}
	// bind canID and s[i]
	IDcan2 = s[2];
	addr_eth.sin_family = AF_INET;
	addr_eth.sin_port = htons(4989);
	addr_eth.sin_addr.s_addr = inet_addr("165.242.111.229");

	for (i=0; i < currmax; i++) {

		ptr = argv[optind+i];
		nptr = strchr(ptr, ',');

		// s[0]:can socket s[1]:can socket s[2]:ethernet socket
		// Create CAN socket in s[i] 
		s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW); //regist to bind
		if (s[i] < 0) {
			perror("socket");
			return 1;
		}

		/* bind IDcan and s[i] */
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

		if (nptr) {

			/* found a ',' after the interface name => check for filters */

			/* determine number of filters to alloc the filter space */
			numfilter = 0;
			ptr = nptr;
			while (ptr) {
				numfilter++;
				ptr++; /* hop behind the ',' */
				ptr = strchr(ptr, ','); /* exit condition */
			}

			rfilter = ((struct can_filter *)malloc((sizeof(struct can_filter))* numfilter));
			if (!rfilter) {
				fprintf(stderr, "Failed to create filter space!\n");
				return 1;
			}
/*,*/
			numfilter = 0;
			err_mask = 0;

			while (nptr) {

				ptr = nptr+1; /* hop behind the ',' */
				nptr = strchr(ptr, ','); /* update exit condition */

				if (sscanf(ptr, "%x:%x",
					   &rfilter[numfilter].can_id, 
					   &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "%x~%x",
						  &rfilter[numfilter].can_id, 
						  &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_id |= CAN_INV_FILTER;
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "#%x", &err_mask) != 1) { 
					fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
					return 1;
				}
			}

			if (err_mask)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
					   &err_mask, sizeof(err_mask));

			if (numfilter)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER,
					   rfilter, numfilter * sizeof(struct can_filter));

			free(rfilter);

		} /* if (nptr) */

		if (rcvbuf_size) {

			int curr_rcvbuf_size;
			socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

			/* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
			if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUFFORCE,
				       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
#ifdef DEBUG
				printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
				if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
					perror("setsockopt SO_RCVBUF");
					return 1;
				}

				if (getsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0) {
					perror("getsockopt SO_RCVBUF");
					return 1;
				}

				/* Only print a warning the first time we detect the adjustment */
				/* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
				if (!i && curr_rcvbuf_size < rcvbuf_size*2)
					fprintf(stderr, "The socket receive buffer size was "
						"adjusted due to /proc/sys/net/core/rmem_max.\n");
			}
		}

		if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}

		/**************************************/
		// error occured: No such device or address
		/*
		if (bind(s[i], (struct sockaddr *)&addr2, sizeof(addr2)) < 0) {
			perror("bind2");
			return 1;
		}
		*/
	}

	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	char INcan0_Interface_table[2][CANID_MAXIMAMNUM];
	char INcan1_Interface_table[2][CANID_MAXIMAMNUM];
	char INcan2_Interface_table[2][CANID_MAXIMAMNUM];
	int can0_masked = 1;
	int can1_masked = 1;
	int can2_masked = 1;
	int j;

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
	 *	    | ・・・  |  ・・・ |	
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
	 *	    | ・・・  |  ・・・ |	
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
	 *	    | ・・・  |  ・・・ |	
	 *   7FD|0 (drop)|0 (drop)|
	 *   7FE|0 (drop)|0 (drop)|
	 *   7FF|0 (drop)|0 (drop)|
	 *
	 */

	//each can_interface filter rule create
	for(i_check=0;i_check<NUMBEROF_MAXIMAMCANID;i_check++) {
		if (canid_filter[i_check].in_interface == I_CAN0 && canid_filter[i_check].pass) {
			if (canid_filter[i_check].can_id == 0) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan0_Interface_table[0][j] = 1;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan0_Interface_table[0][j] = 1;
			} else {
				//id rule
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) INcan0_Interface_table[0][canid_filter[i_check].can_id] = 1;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) INcan0_Interface_table[1][canid_filter[i_check].can_id] = 1;
			}
		} else if (canid_filter[i_check].in_interface == I_CAN1 && canid_filter[i_check].pass) {
			if (canid_filter[i_check].can_id == 0) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan1_Interface_table[0][j] = 1;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan1_Interface_table[0][j] = 1;
			} else {
				//id rule
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) INcan1_Interface_table[0][canid_filter[i_check].can_id] = 1;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) INcan1_Interface_table[1][canid_filter[i_check].can_id] = 1;
			}
		} else if (canid_filter[i_check].in_interface == I_CAN2 && canid_filter[i_check].pass) {
			if (canid_filter[i_check].can_id == 0) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan2_Interface_table[0][j] = 1;
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan2_Interface_table[0][j] = 1;
			} else {
				//id rule
				if (canid_filter[i_check].out_interface1 == I_CAN0 || canid_filter[i_check].out_interface2 == I_CAN0) INcan2_Interface_table[0][canid_filter[i_check].can_id] = 1;
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) INcan2_Interface_table[1][canid_filter[i_check].can_id] = 1;
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

	/* table print */
	/*for (j = 0; j < CANID_MAXIMAMNUM; j++) {
		printf("%d|%d|%d|\t%d|%d|%d|\t%d|%d|%d|\n", j, 
			INcan0_Interface_table[0][j], INcan0_Interface_table[1][j], j,
			INcan1_Interface_table[0][j], INcan1_Interface_table[1][j], j,
			INcan2_Interface_table[0][j], INcan2_Interface_table[1][j]);
	}*/

	//dump main
	while (running) {

		masked=1;

		FD_ZERO(&rdfs);
		for (i=0; i<currmax; i++)
			FD_SET(s[i], &rdfs);

		if ((ret = select(s[currmax-1]+1, &rdfs, NULL, NULL, NULL)) < 0) {
			//perror("select");
			running = 0;
			continue;
		}

		for (i=0; i<currmax; i++) {  /* check all CAN RAW sockets */

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

						if(can1_masked && can2_masked) break;

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

						if(can0_masked && can2_masked) break;

						if 		(!can0_masked) nbytes = write(IDcan0, &frame, sizeof(struct can_frame));
						else if (!can2_masked) nbytes = write_eth(IDcan2, addr_eth, &frame);

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

						if(can0_masked && can1_masked) break;

						if 		(!can0_masked) nbytes = write(IDcan0, &frame, sizeof(struct can_frame));
						else if (!can2_masked) nbytes = write(IDcan1, &frame, sizeof(struct can_frame));

						if 		(!can0_masked) nbytes = write(IDcan0, &frame, sizeof(struct can_frame));
						else if (!can2_masked) nbytes = write(IDcan1, &frame, sizeof(struct can_frame));
				}

				//if(masked) break;
				if (can0_masked && can1_masked && can2_masked);
				
				fprint_long_canframe(stdout, &frame, NULL, view); /* show can-frame */
				printf("\n");
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
