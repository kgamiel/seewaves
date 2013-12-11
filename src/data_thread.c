/*
 * data_thread.c
 *
 *  Created on: Dec 10, 2013
 *      Author: kgamiel
 */

#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <float.h>
#include <math.h>
#include "GL/glfw.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <assert.h>
#include "seewaves.h"
#include "ptp.h"

/*
Data thread loop.  This function is the main loop for the data thread.

- Binds to a UDP port
- Listens for incoming PTP UDP packets from server
- Upon receipt of a packet:
    - Gets mutex lock
    - Updates global data structures
    - Releases mutex lock

@param  user_data   seewaves_t ptr cast to void ptr.

@returns NULL
*/
void *data_thread_main(void *user_data) {
    /* option value */
    int optval = 1;

    /* bind hints */
    struct addrinfo hints;
    struct addrinfo *res;

    /* loop exit variable */
    int done = 0;

    /* return values */
    int err;

    /* port as string */
    char port_as_string[32];

    /* cast to our global data structure pointer */
    seewaves_t *sw = (seewaves_t*)user_data;

    /* setup address */
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    sprintf(port_as_string, "%i", sw->data_port);
    if((err = getaddrinfo(sw->data_host, port_as_string, &hints, &res))) {
       fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(err));
       exit(EXIT_FAILURE);
    }

    /* create server socket */
    if ((sw->data_socket_fd = socket(res->ai_family, res->ai_socktype,
        res->ai_protocol)) == -1) {
        perror("socket");
        pthread_exit(NULL);
    }

    /* reuse local address */
    if (setsockopt(sw->data_socket_fd, SOL_SOCKET, SO_REUSEADDR, &optval,
                  sizeof optval) == -1) {
        perror("setsockopt(SO_REUSEADDR)");
        close(sw->data_socket_fd);
        pthread_exit(NULL);
    }

#ifdef SO_REUSEPORT
    /* reuse local port (if available) */
    if (setsockopt(sw->data_socket_fd, SOL_SOCKET, SO_REUSEPORT, &optval,
                  sizeof optval) == -1) {
        perror("setsockopt(SO_REUSEPORT)");
        close(sw->data_socket_fd);
        pthread_exit(NULL);
    }
#endif


    /* bind to local address:port */
    if (bind(sw->data_socket_fd, res->ai_addr, res->ai_addrlen) == -1) {
        perror("bind");
        pthread_exit(NULL);
    }

    /* Optionally set maximum UDP receiver buffer size */
    if(sw->udp_buffer_size > 0) {
    	if (setsockopt(sw->data_socket_fd, SOL_SOCKET, SO_RCVBUF, &sw->udp_buffer_size,
    			(socklen_t)(sizeof(int))) == -1) {
    		perror("setsockopt(SO_RCVBUF)");
    	}
    }

    /* get actual UDP receive buffer size in use */
    sw->udp_buffer_size = util_get_udp_buffer_size(sw->data_socket_fd);

    /* enable non-blocking */
    fcntl(sw->data_socket_fd, F_SETFL, O_NONBLOCK);

    /* Loop until application asks us to exit */
    while(!done) {
        struct sockaddr data_socket_remote_address;
        socklen_t data_socket_remote_address_len;

        /* packet variable */
        ptp_packet_t packet;

        /* length of packet received */
        ssize_t packet_length_bytes;

        /* receive a packet */
        memset((char *) &data_socket_remote_address, 0,
               sizeof(data_socket_remote_address));
        packet_length_bytes = recvfrom(sw->data_socket_fd, &packet,
                                       sizeof(ptp_packet_t),
                                       0, (struct sockaddr *)
                                       &data_socket_remote_address,
                                       &data_socket_remote_address_len);
        /* did we receive a packet? */
        int new_model_received = 0;
        if (packet_length_bytes == sizeof(ptp_packet_t)) {
            /* yes, get mutex lock */
            int locked = 0;
            while(!locked) {
                if ((err = pthread_mutex_trylock(&sw->lock)) == 0) {
                    /* got the lock */
                    unsigned int particle = 0;
                    /* exit loop flag */
                    locked++;

                    /* KAG - fix me, should be list, they can be out-of-order
                     * keep most recent timestamp */
                    if(packet.t > sw->most_recent_timestamp) {
                    	sw->most_recent_timestamp = packet.t;
                    	sw->total_timesteps++;
                    }

                    /* keep track of packet count received */
                    sw->packets_received++;

                    /* allocate memory if first time or new particle count */
                    if (sw->total_particle_count !=
                        packet.total_particle_count) {
                    	new_model_received = 1;
                    	if (sw->x != NULL) {
                            /* not first time, but different count, so free */
                            free(sw->x);
                            free(sw->y);
                            free(sw->z);
                            free(sw->flag);
                            free(sw->t);
                        }
                        sw->x = (double*)calloc(packet.total_particle_count,
                            sizeof(double));
                        sw->y = (double*)calloc(packet.total_particle_count,
                            sizeof(double));
                        sw->z = (double*)calloc(packet.total_particle_count,
                            sizeof(double));
                        sw->particle_type = (short*)calloc(packet.total_particle_count,
                            sizeof(short));
                        sw->t = (float*)calloc(packet.total_particle_count,
                            sizeof(float));
                        for(particle = 0; particle < packet.total_particle_count;particle++) {
                        	sw->x[particle] = UNDEFINED_PARTICLE;
                        }
                        sw->rotation_center[0] = UNDEFINED_PARTICLE;
                        memcpy(sw->world_origin, packet.world_origin, sizeof(packet.world_origin));
                        memcpy(sw->world_size, packet.world_size, sizeof(packet.world_size));
                    }

                    /* save total number of particles in model */
                    sw->total_particle_count = packet.total_particle_count;

                    /* loop through particles in this packet */
                    for(; particle < packet.particle_count; particle++) {
                        /* get particle id */
                        unsigned int id = packet.data[particle].id;
                        /* set x, y, z and w */
                        sw->t[id] = packet.t;
                        sw->x[id] = packet.data[particle].position[0];
                        sw->y[id] = packet.data[particle].position[1];
                        sw->z[id] = packet.data[particle].position[2];
                        sw->particle_type[id] = packet.data[particle].particle_type;
                        /*sw->w[id] = packet.data[particle].position[2];*/
                        /*sw->flag[id] = packet.data[particle].flag;*/
                    }
                    if(sw->rotation_center[0] == UNDEFINED_PARTICLE) {
                    	sw->rotation_center[0] = sw->world_origin[0] + sw->world_size[0] / 2.0;
                    	sw->rotation_center[1] = sw->world_origin[2] + sw->world_size[2] / 2.0;
                    	sw->rotation_center[2] = sw->world_origin[1] + sw->world_size[1] / 2.0;
                    }
                    sw->udp_buffer_size = sw->total_particle_count * sizeof(ptp_packet_t);
                    /*
    	            if (setsockopt(sw->data_socket_fd, SOL_SOCKET, SO_RCVBUF,
                        &sw->udp_buffer_size,
    			        (socklen_t)(sizeof(int))) == -1) {
    		            perror("setsockopt(SO_RCVBUF)");
                        printf("Unable to allocated %i bytes\n",
                            sw->udp_buffer_size);
                    }
                    */
#ifdef XXX
                    if(new_model_received) {
                        /* set camera based on new world */
                        GLfloat eye[3];
                        GLfloat target[3];
                        eye[0] = g_seewaves.world_origin[0] + (g_seewaves.world_size[0] / 2);
                        eye[1] = g_seewaves.world_origin[2] + (g_seewaves.world_size[2] / 2);
                        eye[2] = g_seewaves.world_origin[1] + (g_seewaves.world_size[1] / 2);
                        target[0] = g_seewaves.world_origin[0] + (g_seewaves.world_size[0] / 2);
                        target[1] = g_seewaves.world_origin[2] + (g_seewaves.world_size[2] / 2);
                        target[2] = g_seewaves.world_origin[1] + (g_seewaves.world_size[1] / 2);
                        camera_set_raw(eye[0], eye[1], eye[2], 0.0, 1.0, 0.0, target[0], target[1], target[2]);

                    	new_model_received = 0;
                    }
#endif
                    /* Release the lock */
                    if ((err = pthread_mutex_unlock(&sw->lock))) {
                        fprintf(stderr, "Error creating mutex: %i\n", err);
                    }
                } else {
                    if(err == EBUSY) {
                        /* mutex was locked */
                        printf("!LOCK\n");
                        fflush(stdout);
                    } else if(err == EINVAL) {
                        fprintf(stderr, "Invalid mutex lock?");
                    } else {
                        fprintf(stderr, "Unhandled mutex lock error: %i", err);
                    }
                }
            }
        } else if (packet_length_bytes == 0) {
            /* socket closed on linux */
            done = 1;
        } else {
            if(errno == EINTR) {
                /* ignore */
            } else if (errno == EBADF) {
                /* Parent thread closed the socket, that's our signal that
                we're done */
                done = 1;
            } else if (errno == EINVAL) {
                /* Invalid argument */
                done = 1;
            } else if (errno == EAGAIN) {
                /* no data available, */
            } else if (errno == EWOULDBLOCK) {
                /* ignore */
            } else {
                printf("%i\n", errno);
                perror("data recvfrom");
                done = 1;
            }
        }
        //usleep(10);
    }
    if(sw->verbosity) {
        printf("Data thread exiting\n");
        fflush(stdout);
    }

    /* close our socket */
    close(sw->data_socket_fd);

    /* we're done */
    return(NULL);
}

