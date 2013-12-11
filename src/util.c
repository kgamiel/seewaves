/*
 * util.c
 *
 *  Created on: Dec 10, 2013
 *      Author: kgamiel
 */

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <time.h>
#include "util.h"

/*
Utility function to get UDP receiver buffer size.

@param	sd	socket descriptor or -1 for system default

@returns size in bytes
*/
int util_get_udp_buffer_size(int sd) {
	int size;
	int len = sizeof(int);

	if(sd == -1) {
		int fd;
		if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
			perror("socket");
			return(0);
		}
		if (getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &size, (socklen_t*)&len) == -1) {
			perror("getsockopt(bufsize)");
			close(fd);
		}
		return(size);
	}
	if (getsockopt(sd, SOL_SOCKET, SO_RCVBUF, &size, (socklen_t*)&len) == -1) {
		perror("getsockopt(bufsize)");
	}
	return(size);
}

/*
Get current date/time as a formatted string.

@param	buf	Buffer into which string is stored.
@param	max_len	Size of buffer
*/
void util_get_current_time_string(char *buf, ssize_t max_len) {
	time_t now;
	struct tm * timeinfo;
	time(&now);
	timeinfo = localtime(&now);
	strftime(buf, max_len, "%m-%d-%Y_%X", timeinfo);
}

