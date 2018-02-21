/*
 *  $Id$
 */

/*
 * can3way_child.c - can3-waytransfer.c used in combination
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.c"

int attackSend_CountorTime(int *s, int *nbyte, struct can_frame frame, int attack_count, double attack_cycle, double attack_time);

int main(int argc, char **argv)
{
	/*argv[1]:can0, argv[2]:Data, argv[3]:Count, argv[4]:Cycle(us), argv[5]:Time(us)*/
	int s_can; /* can raw socket */ 
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	/* check command line options */
	/*
	if (argc != 6) {
		fprintf(stderr, "Usage: %s <device> <can_frame> <Data> <Count> <Cycle> <Time>.\n", argv[0]);
		return 1;
	}*/
	
	int s_udp;
	struct sockaddr_in addr_udp;
	socklen_t sin_size = sizeof(struct sockaddr_in);
	struct sockaddr_in from_addr;

	char buf[1024];

	if ((s_udp = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		return 1;
	}

	addr_udp.sin_family = AF_INET;
	addr_udp.sin_port = htons(4989);
	addr_udp.sin_addr.s_addr = INADDR_ANY;

	if (bind(s_udp, (struct sockaddr *)&addr_udp, sizeof(addr_udp)) < 0) {
		perror("bind");
		return 1;
	}

	memset(buf, 0, sizeof(buf));
	/* open socket */
	if ((s_can = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, "can0");
	if (ioctl(s_can, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	int running = 1;

	printf("eth dump running\n");
	while (running) {
		printf("waiting eth packet\n");
		if (recv(s_udp, buf, sizeof(buf), 0) < 0) {
			perror("recv");
			return 1;
		}
		printf("%s\n",buf);

		/*Divide received data */
		char *CANData, *attack_count, *attack_cycle, *attack_time, *cansendinfo[255];
		int i = 0;
		cansendinfo[0] = strtok(buf, ":");
		do {
			cansendinfo[++i] = strtok(NULL, ":");
		} while ((i < 4) && (cansendinfo[i] != NULL));

		CANData = cansendinfo[0];
		attack_count = cansendinfo[1];
		attack_cycle = cansendinfo[2];
		attack_time = cansendinfo[3];
		/* parse CAN frame */
		if (parse_canframe(CANData, &frame)){
			fprintf(stderr, "\nWrong CAN-frame format!\n\n");
			fprintf(stderr, "Try: <can_id>#{R|data}\n");
			fprintf(stderr, "can_id can have 3 (SFF) or 8 (EFF) hex chars\n");
			fprintf(stderr, "data has 0 to 8 hex-values that can (optionally)");
			fprintf(stderr, " be seperated by '.'\n\n");
			fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / ");
			fprintf(stderr, "5AA# /\n     1F334455#1122334455667788 / 123#R ");
			fprintf(stderr, "for remote transmission request.\n\n");
			return 1;
		}


		/* disable default receive filter on this RAW socket */
		/* This is obsolete as we do not read from the socket at all, but for */
		/* this reason we can remove the receive list in the Kernel to save a */
		/* little (really a very little!) CPU usage.                          */
		setsockopt(s_can, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

		if (bind(s_can, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}
		
		attackSend_CountorTime(&s_can, &nbytes, frame, 1, 0 ,0);
	}

	close(s_udp);
	close(s_can);
	return 0;
}

int attackSend_CountorTime(int *s, int *nbytes, struct can_frame frame, int attack_count, double attack_cycle, double attack_time) {
	/* When attack count is more than zero, Count attack execute */
	/* otherwise,  Cycle attack execute */
	if (attack_time/(1000*1000*1000) > 60) attack_time = 0;
	struct timespec attack_Cycle;
	attack_Cycle.tv_sec = 0;
	attack_Cycle.tv_nsec = attack_cycle;
	if (attack_count <= 0) {
		/* Cycle attack*/
		struct timespec start, end;
		clock_gettime(CLOCK_REALTIME, &start);
		end = start;
		attack_time /= 1000;
		double dur = (end.tv_sec - start.tv_sec)*(1000*1000) + (end.tv_nsec - start.tv_nsec)/1000;
		while (dur < attack_time) {
			/* send frame */
			if ((*nbytes = write(*s, &frame, sizeof(frame))) != sizeof(frame)) {
				perror("write");
				return 1;
			}
			clock_nanosleep(CLOCK_REALTIME, 0, &attack_Cycle, NULL);
			clock_gettime(CLOCK_REALTIME, &end);
			dur = (end.tv_sec - start.tv_sec)*(1000*1000) + (end.tv_nsec - start.tv_nsec)/1000;
		}
	} else {
		/*Count attack*/
		int i;
		for (i = 0; i < attack_count; i++) {
			/* send frame */
			if ((*nbytes = write(*s, &frame, sizeof(frame))) != sizeof(frame)) {
				perror("write");
				return 1;
			}
			clock_nanosleep(CLOCK_REALTIME, 0, &attack_Cycle, NULL);
		}
	}
}
