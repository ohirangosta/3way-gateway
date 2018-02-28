/*
 *  $Id$
 */

/*
 * can3way-gateway-child.c - can3-waytransfer.c used in combination
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

#define CAN_RECV_SOCKET 0
#define UDP_RECV_SOCKET 1
#define ANYDEV "any"

int write_eth (int socket, struct sockaddr_in addr, struct can_frame *frame) {
	char send_data[100];
	sprint_canframe(send_data, frame, 0);
	if (sendto(socket, send_data, strlen(send_data), 0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("sendto_eth");
		return -1;
	}
#ifdef DEBUG
	printf("[dump CAN] %s\n", send_data);
#endif
	return 0;
}

int main(int argc, char **argv)
{
	fd_set rdfs;
	int i;
	int ret;
	int s_udp_send; /* udp send socket */ 
	int s[2]; /* s[0] is can socket, s[1] is udp recv socket*/
	int currmax = 2;
	int nbytes;
	struct sockaddr_can addr_can;
	struct can_frame frame;
	struct ifreq ifr;
	struct sockaddr_in addr_udp_send;
	struct sockaddr_in addr_udp_recv;
	socklen_t sin_size = sizeof(struct sockaddr_in);
	struct sockaddr_in from_addr;
	char udp_recv_buf[1024];
	struct iovec iov;
	struct msghdr msg;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	int max_sock;
	char *ptr = "can0";
	char *nptr;

	if ((s_udp_send = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		return 1;
	}
	if ((s[1] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		return 1;
	}

	addr_udp_send.sin_family = AF_INET;
	addr_udp_send.sin_port = htons(9894);
	addr_udp_send.sin_addr.s_addr = inet_addr("169.254.115.249");//ip address 169.254.115.249 is 3way-transfer-parent
	addr_udp_recv.sin_family = AF_INET;
	addr_udp_recv.sin_port = htons(4989);
	addr_udp_recv.sin_addr.s_addr = INADDR_ANY;

	if (bind(s[1], (struct sockaddr *)&addr_udp_recv, sizeof(addr_udp_recv)) < 0) {
		perror("udp recv socket bind");
		return 1;
	}

	memset(udp_recv_buf, 0, sizeof(udp_recv_buf));

	/* open can socket */
	nptr = strchr(ptr, ',');
	if ((s[0] = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}
	if (nptr) nbytes = nptr -ptr;
	else nbytes = strlen(ptr);

	if (nbytes >= IFNAMSIZ) {
		fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
		return 1;
	}

	addr_can.can_family = AF_CAN;

	memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	strncpy(ifr.ifr_name, ptr, nbytes);
	if (strcmp(ANYDEV, ifr.ifr_name)) {
		if (ioctl(s[0], SIOCGIFINDEX, &ifr) < 0) {
			perror("SIOCGIFINDEX");
			return 1;
		}
		addr_can.can_ifindex = ifr.ifr_ifindex;
	} else addr_can.can_ifindex = 0;


	if (bind(s[0], (struct sockaddr *)&addr_can, sizeof(addr_can)) < 0) {
		perror("bind");
		return 1;
	}

	iov.iov_base = &frame;
	msg.msg_name = &addr_can;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;
	
	if (s[0] > s[1]) max_sock = s[0];
	else max_sock = s[1];
			
	int running = 1;
#ifdef DEBUG
	printf("can and eth dump running\n");
#endif
	while (running) {
		FD_ZERO(&rdfs);
		for (i = 0; i < currmax; i++)
			FD_SET(s[i], &rdfs);
		
		select(max_sock+1, &rdfs, NULL, NULL, NULL);
		for (i = 0; i < currmax; i++) {
			if (FD_ISSET(s[i], &rdfs)) {

				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr_can);
				msg.msg_controllen = sizeof(ctrlmsg);
				msg.msg_flags = 0;
				
				if (i == CAN_RECV_SOCKET) {
					nbytes = recvmsg(s[i], &msg, 0);
					if (nbytes < 0) {
						perror("can read");
						return 1;
					}
					if (nbytes < sizeof(struct can_frame)) {
						fprintf(stderr, "can read: incomplete CAN frame\n");
						return 1;
					}
					nbytes = write_eth(s_udp_send, addr_udp_send, &frame);
				} else if (i == UDP_RECV_SOCKET) {
					if (recv(s[i], udp_recv_buf, sizeof(udp_recv_buf), 0) < 0) {
						perror("udp recv");
						return 1;
					}
					/* From UDP packet to CAN frame*/
					parse_canframe(udp_recv_buf, &frame);
					/* disable default receive filter on this RAW socket */
					/* This is obsolete as we do not read from the socket at all, but for */
					/* this reason we can remove the receive list in the Kernel to save a */
					/* little (really a very little!) CPU usage.                          */
					//attackSend_CountorTime(&s_can, &nbytes, frame, 1, 0 ,0);
					nbytes = write(s[0], &frame, sizeof(frame));
#ifdef DEBUG
					printf("[dump ETH] %s\n",udp_recv_buf);
#endif
				}
			}

		}
	}

	close(s_udp_send);
	close(s[0]);
	close(s[1]);
	return 0;

}
