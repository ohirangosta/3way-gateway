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
 * 2)  ./can3-waytransfer -f config.conf : can packet(s) are filtred and transfered engaged in config file
 *
 * how to compile:
 * ex) gcc cant3-wayransfer.c parser.c lexer.c -lib.c -o cant3-wayransfer
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
	return i+1;
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

	//test
	/*
	canid_filter[0].can_id = 0x0B4;
	canid_filter[0].pass = 1;
	canid_filter[1].can_id = 0x0AA;
	canid_filter[1].pass = 1;
	*/

	/****************************/

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	last_tv.tv_sec  = 0;
	last_tv.tv_usec = 0;

	/* default add Option: -B */
	
	argv[argc++] = "can0";
	argv[argc++] = "can1";
	argv[argc++] = "-B";
	argv[argc++] = "can1";
	//argv[argc++] = "can1";
	
/*
	argv[argc++] = {(const char *)"can0"};
	argv[argc++] = {(const char *)"can1"};
	argv[argc++] = {(const char *)"-B"};
	argv[argc++] = {(const char *)"can1"};
*/
	for(i=0;i<argc;i++)	printf("%d:%s\n",i,argv[i]);

	char **argv_tmp;
	argv_tmp = argv;

	while ((opt = getopt(argc, argv, "t:ciaSs:b:B:u:ldLn:r:f:Dhe?")) != -1) {
		switch (opt) {

		case 'a':
			view |= CANLIB_VIEW_ASCII;
			break;

		case 'e':
			view |= CANLIB_VIEW_ERROR;
			break;

		case 's':
			silent = atoi(optarg);
			if (silent > SILENT_ON) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'b':
		case 'B':
			/* Bridge: about sending */
			if (strlen(optarg) >= IFNAMSIZ) {
				fprintf(stderr, "Name of CAN device '%s' is too long!\n\n", optarg);
				return 1;
			} else {
				bridge = socket(PF_CAN, SOCK_RAW, CAN_RAW);
				bridge_re = socket(PF_CAN, SOCK_RAW, CAN_RAW); /* for retransfer */
				if (bridge < 0) {
					perror("bridge socket");
					return 1;
				}
				addr.can_family = AF_CAN;
				addr2.can_family = AF_CAN;
				strcpy(ifr.ifr_name, optarg);//can1
				strcpy(ifr2.ifr_name, argv[2]);//can0 /*copy for tansfer*/

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

				/*for tansfer*/
				/*
				if (bind(bridge_re, (struct sockaddr *)&addr2, sizeof(addr2)) < 0) {
					perror("bridge bind2");
					return 1;
				}*/
			}
			break;
	    
		case 'u':
			bridge_delay = (useconds_t)strtoul(optarg, (char **)NULL, 10);
			break;

		case 'l':
			log = 1;
			break;

		case 'd':
			dropmonitor = 1;
			break;

		case 'n':
			count = atoi(optarg);
			if (count < 1) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'r':
			rcvbuf_size = atoi(optarg);
			if (rcvbuf_size < 1) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		/* load a config file */
		case 'f':
			apply_rule = 1;
			// 3-way add
			yyin = fopen(argv[2], "r");
			init_buf();
			struct AbstSyntaxTree *syntax_rule = parse();
			rule_counter = merge_parser2can_filter_rule(syntax_rule, canid_filter);
			printf("syntax OK!!!!!!!!\n");
			print_rule(syntax_rule);
			//printf("CONFIG CANID:%x PASS:%d IN_INTERFACE:%d OUT1:%d OUT2:%d\n",canid_filter[0].can_id, canid_filter[0].pass, canid_filter[0].in_interface, canid_filter[0].out_interface1, canid_filter[0].out_interface2);
			break;

		case 'D':
			DEBUG_MODE=1;
			break;

		default:
			print_usage(basename(argv[0]));
			exit(1);
			break;
		}
	}//end of optional while

	if (optind == argc) {
		print_usage(basename(argv[0]));
		exit(0);
	}
	
	if (logfrmt && view) {
		fprintf(stderr, "Log file format selected: Please disable ASCII/BINARY/SWAP options!\n");
		exit(0);
	}

	if (silent == SILENT_INI) {
		if (log) {
			fprintf(stderr, "Disabled standard output while logging.\n");
			silent = SILENT_ON; /* disable output on stdout */
		} else
			silent = SILENT_OFF; /* default output */
	}

	currmax = argc - optind; /* find real number of CAN devices */


	if (currmax > MAXSOCK) {
		fprintf(stderr, "More than %d CAN devices given on commandline!\n", MAXSOCK);
		return 1;
	}

/****************************for can2*************************/
	/* UDP server for can2 */
	struct sockaddr_in addr_eth;
	if ((s[2] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("eth socket");
		return 1;
	}
	/* bind canID and s[i] */
	IDcan2 = s[2];

	addr_eth.sin_family = AF_INET;
	addr_eth.sin_port = htons(4989);
	addr_eth.sin_addr.s_addr = inet_addr("169.254.19.16");
/*************************************************************/

	for (i=0; i < currmax; i++) {

		ptr = argv[optind+i];
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

		//memset(&ifr2.ifr_name, 0, sizeof(ifr2.ifr_name)); /* for transfer */
		//strncpy(ifr2.ifr_name, argv[3], nbytes); 		  /* for transfer*/

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

		if (timestamp || log || logfrmt) {

			const int timestamp_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMP,
				       &timestamp_on, sizeof(timestamp_on)) < 0) {
				perror("setsockopt SO_TIMESTAMP");
				return 1;
			}
		}

		if (dropmonitor) {

			const int dropmonitor_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_RXQ_OVFL,
				       &dropmonitor_on, sizeof(dropmonitor_on)) < 0) {
				perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
				return 1;
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

	if (log) {
		time_t currtime;
		struct tm now;
		char fname[sizeof("candump-2006-11-20_202026.log")+1];

		if (time(&currtime) == (time_t)-1) {
			perror("time");
			return 1;
		}

		localtime_r(&currtime, &now);

		sprintf(fname, "candump-%04d-%02d-%02d_%02d%02d%02d.log",
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);

		if (silent != SILENT_ON)
			printf("\nWarning: console output active while logging!");

		fprintf(stderr, "\nEnabling Logfile '%s'\n\n", fname);

		logfile = fopen(fname, "w");
		if (!logfile) {
			perror("logfile");
			return 1;
		}
	}

	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	/*for dump*//*
	iov2.iov_base = &frame2;
	msg2.msg_name = &add2r;
	msg2.msg_iov = &iov2;
	msg2.msg_iovlen = 1;
	msg2.msg_control = &ctrlmsg;*/

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
	for(i_check=0;i_check<rule_counter-1;i_check++) {
		if (canid_filter[i_check].in_interface == I_CAN0) {
			if (canid_filter[i_check].can_id == 2048) {
				//default rule
				if (canid_filter[i_check].out_interface1 == I_CAN1 || canid_filter[i_check].out_interface2 == I_CAN1) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan0_Interface_table[0][j] = canid_filter[i_check].pass_drop;
				if (canid_filter[i_check].out_interface1 == I_CAN2 || canid_filter[i_check].out_interface2 == I_CAN2) for (j = 0; j < CANID_MAXIMAMNUM; j++) INcan0_Interface_table[1][j] = canid_filter[i_check].pass_drop;
			} else {
				//id rule
				printf("id rule %d - %d %d\n", canid_filter[i_check].can_id_range_start, canid_filter[i_check].can_id_range_end, canid_filter[i_check].can_id);
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

		masked=1;

		FD_ZERO(&rdfs);
		for (i=0; i<currmax + ThreeWAYGW_eth; i++)
			FD_SET(s[i], &rdfs);

		if ((ret = select(s[currmax-1]+1, &rdfs, NULL, NULL, NULL)) < 0) {
			//perror("select");
			running = 0;
			continue;
		}

		for (i=0; i<currmax + ThreeWAYGW_eth; i++) {  /* check all CAN RAW sockets */

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

				if (count && (--count == 0))
					running = 0;

				if (bridge) {/*transfer*/
					if (bridge_delay)
						usleep(bridge_delay);

					//mask can_id
					//printf("(DEBUG - FRAME CHECK) ID:%x , DLC:%d, DATA[0]:%x [1]:%x [2]:%x [3]:%x [4]:%x [5]:%x [6]:%x [7]:%x\n",frame.can_id,frame.can_dlc,frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
					//if(frame.can_id == 0xAA) puts("A");
					//for(i_check=0;i_check<NUMBEROFCANID , mask_id[i_check] != 0  ;i_check++) if(strtol(can_id, NULL, 16) == mask_id[i_check]) masked = 0;

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

							if(can1_masked && can2_masked) break;

							if (!can1_masked) nbytes = write(IDcan1, &frame, sizeof(struct can_frame));
							//if (!can2_masked) nbytes = write_eth(IDcan2, addr_eth, &frame);
							if (1) nbytes = write_eth(IDcan2, addr_eth, &frame);

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

					//continue without buffer problem? if remove below code
					/*if (nbytes < 0) {
						perror("bridge write");
						return 1;
					} else if (nbytes < sizeof(struct can_frame)) {
						fprintf(stderr,"bridge write: incomplete CAN frame\n");
						return 1;
					}*/
				}

				//if(masked) break;
				if (can0_masked && can1_masked && can2_masked);
		    
				for (cmsg = CMSG_FIRSTHDR(&msg);
				     cmsg && (cmsg->cmsg_level == SOL_SOCKET);
				     cmsg = CMSG_NXTHDR(&msg,cmsg)) {
					if (cmsg->cmsg_type == SO_TIMESTAMP)
						tv = *(struct timeval *)CMSG_DATA(cmsg);
					else if (cmsg->cmsg_type == SO_RXQ_OVFL)
						dropcnt[i] = *(__u32 *)CMSG_DATA(cmsg);
				}

				/* check for (unlikely) dropped frames on this specific socket */
				if (dropcnt[i] != last_dropcnt[i]) {

					__u32 frames;

					if (dropcnt[i] > last_dropcnt[i])
						frames = dropcnt[i] - last_dropcnt[i];
					else
						frames = 4294967295U - last_dropcnt[i] + dropcnt[i]; /* 4294967295U == UINT32_MAX */

					if (silent != SILENT_ON)
						printf("DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
						       frames, (frames > 1)?"s":"", cmdlinename[i], dropcnt[i]);

					if (log)
						fprintf(logfile, "DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
							frames, (frames > 1)?"s":"", cmdlinename[i], dropcnt[i]);

					last_dropcnt[i] = dropcnt[i];
				}

				idx = idx2dindex(addr.can_ifindex, s[i]);

				if (log) {
					/* log CAN frame with absolute timestamp & device */
					fprintf(logfile, "(%ld.%06ld) ", tv.tv_sec, tv.tv_usec);
					fprintf(logfile, "%*s ", max_devname_len, devname[idx]);
					/* without seperator as logfile use-case is parsing */
					fprint_canframe(logfile, &frame, "\n", 0);
				}

				if (logfrmt) {
					/* print CAN frame in log file style to stdout */
					printf("(%ld.%06ld) ", tv.tv_sec, tv.tv_usec);
					printf("%*s ", max_devname_len, devname[idx]);
					fprint_canframe(stdout, &frame, "\n", 0);
					goto out_fflush; /* no other output to stdout */
				}

				if (silent != SILENT_OFF){
					if (silent == SILENT_ANI) {
						printf("%c\b", anichar[silentani%=MAXANI]);
						silentani++;
					}
					goto out_fflush; /* no other output to stdout */
				}
		      
				printf(" %s", (color>2)?col_on[idx%MAXCOL]:"");

				switch (timestamp) {

				case 'a': /* absolute with timestamp */
					printf("(%ld.%06ld) ", tv.tv_sec, tv.tv_usec);
					break;

				case 'A': /* absolute with date */
				{
					struct tm tm;
					char timestring[25];

					tm = *localtime(&tv.tv_sec);
					strftime(timestring, 24, "%Y-%m-%d %H:%M:%S", &tm);
					printf("(%s.%06ld) ", timestring, tv.tv_usec);
				}
				break;

				case 'd': /* delta */
				case 'z': /* starting with zero */
				{
					struct timeval diff;

					if (last_tv.tv_sec == 0)   /* first init */
						last_tv = tv;
					diff.tv_sec  = tv.tv_sec  - last_tv.tv_sec;
					diff.tv_usec = tv.tv_usec - last_tv.tv_usec;
					if (diff.tv_usec < 0)
						diff.tv_sec--, diff.tv_usec += 1000000;
					if (diff.tv_sec < 0)
						diff.tv_sec = diff.tv_usec = 0;
					printf("(%03ld.%06ld) ", diff.tv_sec, diff.tv_usec);
				
					if (timestamp == 'd')
						last_tv = tv; /* update for delta calculation */
				}
				break;

				default: /* no timestamp output */
					break;
				}

				printf(" %s", (color && (color<3))?col_on[idx%MAXCOL]:"");
				printf("%*s", max_devname_len, devname[idx]);
				printf("%s  ", (color==1)?col_off:"");
				
				if(DEBUG_MODE) printf("s[i]=%d (can0=%d, can1=%d)\n",s[i],IDcan0,IDcan1);
				
				fprint_long_canframe(stdout, &frame, NULL, view); /* show can-frame */

				printf("%s", (color>1)?col_off:"");
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

	if (log)
		fclose(logfile);

	return 0;
}
