/*
 * heartbeat.c
 *
 *  Created on: Dec 10, 2013
 *      Author: kgamiel
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "ptp.h"
#include "heartbeat.h"

/*
Heartbeat thread loop.
This function is the main loop for the heartbeat thread.  It sends heartbeat
packet to server at regular intervals.

@param  user_data   seewaves_t ptr cast to void ptr.

@returns NULL
*/
void *heartbeat_thread_main(void *seewaves_data) {
    /* address variables */
    struct sockaddr_in heartbeat_socket_remote_address;
    socklen_t heartbeat_socket_remote_address_len = sizeof(
                heartbeat_socket_remote_address);

    /* hints to resolver */
    struct addrinfo address_hints;
    struct addrinfo *address_p;

    /* server address information returned from resolver */
    struct addrinfo *server_address_info;

    /* port number as string */
    char port_as_string[16];

    /* return values */
    long err;

    /* heartbeat packet */
    ptp_heartbeat_packet_t hb;

    /* current time */
    time_t now;

    /* last time a heartbeat was sent to server */
    time_t last_heartbeat_sent;

    /* loop flag */
    int done = 0;

    /* cast to our global data structure pointer */
    seewaves_t *sw = (seewaves_t*)seewaves_data;

    /* create socket */
    if ((sw->heartbeat_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) ==
        -1) {
        perror("socket");
        pthread_exit(NULL);
    }

    /* set non-blocking so we can use it to test for closure */
    fcntl(sw->heartbeat_socket_fd, F_SETFL, O_NONBLOCK);

    /* resolve host name */
    memset(&address_hints, 0, sizeof address_hints); /* clear the struct */
    address_hints.ai_family = AF_UNSPEC;     /* IPv4 or IPv6 */
    address_hints.ai_socktype = SOCK_DGRAM;  /* UDP */
    sprintf(port_as_string, "%i", sw->gpusph_port);
    if ((err = getaddrinfo(sw->gpusph_host, port_as_string,
        &address_hints, &server_address_info))) {
        fprintf(stderr, "getaddrinfo() failed: %s", gai_strerror((int)err));
        close(sw->heartbeat_socket_fd);
        pthread_exit(NULL);
    }

    /* loop over returned IP addresses, just use the first IPv4 */
    for (address_p = server_address_info; address_p != NULL;
        address_p = address_p->ai_next) {

        void *addr;
        if (address_p->ai_family == AF_INET) { /* IPv4 */
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)address_p->ai_addr;
            addr = &(ipv4->sin_addr);
            memcpy(&heartbeat_socket_remote_address, address_p->ai_addr,
                (size_t)heartbeat_socket_remote_address_len);
            heartbeat_socket_remote_address_len = address_p->ai_addrlen;
            inet_ntop(address_p->ai_family, addr, sw->gpusph_host,
                sizeof(sw->gpusph_host));
            break;
        } else { /* IPv6 */
            struct sockaddr_in6 *ipv6 =
                (struct sockaddr_in6 *)address_p->ai_addr;
            addr = &(ipv6->sin6_addr);
        }
    }

    /* free linked-list returned from resolver */
    freeaddrinfo(server_address_info);

    /* clear heartbeat packet */
    memset(&hb, 0, sizeof(ptp_heartbeat_packet_t));

    /* initialize timing */
    last_heartbeat_sent = 0;

    /* main thread loop */
    while(!done) {
        /* buffer used to test for socket closure */
        unsigned char b[64];

        /* get current time */
        time(&now);
        /* is it time to send a heartbeat? */
        if (difftime(now, last_heartbeat_sent) > PTP_HEARTBEAT_TTL_S) {
            /* yes, send a heartbeat */
            ssize_t bytes_sent;
            bytes_sent = sendto(sw->heartbeat_socket_fd, &hb,
                sizeof(ptp_heartbeat_packet_t), 0,
                (const struct sockaddr *)(&heartbeat_socket_remote_address),
                heartbeat_socket_remote_address_len);
            if (bytes_sent == -1) {
                /* failed.  If EBADF, we'll detect that below  */
                if (errno != EBADF) {
                    /* hmm, real failure of some sort */
                    perror("sendto()");
                }
            } else if(bytes_sent == 0) {
                done = 1;
            } else {
                /* keep track of heartbeats sent */
                sw->heartbeats_sent++;
            }
            /* update timing */
            time(&last_heartbeat_sent);
        }
        /* has main thread closed our socket? */
        err = recvfrom(sw->heartbeat_socket_fd, &b, sizeof(b), 0, NULL, NULL);
        if (err == 0) {
            /* socket closed on linux */
            done = 1;
        } else {
            if(errno == EAGAIN) {
                /* ignore, we're non-blocking and nothings ready */
            } else if(errno == EINTR) {
                /* ignore */
            } else if (errno == EBADF) {
                /* Parent thread closed the socket, that's our signal that
                we're done */
                done = 1;
            } else if (errno == ETIMEDOUT) {
                /* ignore */
            } else {
                perror("heartbeat recvfrom");
                done = 1;
            }
        }
        /* give cpu a break */
        //usleep(10);
    }
    if(sw->verbosity) {
        fprintf(stdout, "Heartbeat thread exiting\n");
        fflush(stdout);
    }

    /* close our socket */
    close(sw->heartbeat_socket_fd);

    /* we're done */
    return(NULL);
}
