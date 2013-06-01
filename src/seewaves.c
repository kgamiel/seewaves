/*============================================================================
Name        : seewaves.cpp
Author      : Kevin Gamiel <kgamiel@renci.org>
Copyright   : Copyright (c) 2013, RENCI
Description : Simple GPUSPH visualization client
Application : seewaves
Purpose     : Seewaves connects to a PTP (particle transport protocol) server
              and renders 3D particles in near-realtime.  The primary purpose
              is to monitor the status of 3D particle simulations running on a
              headless server with minor impact on simulation performance.
Strategy    : PTP uses User Datagram Protocol (UDP) over Internet Protocol (IP)
              and may therefore gracefully lose packets during operation.
              Seewaves, therefore, may start and stop at different times from
              different locations on the network and join a particle simulation
              in progress.
              Seewaves is a multi-threaded application consisting of three
              threads; main, heartbeat and data:
                - main thread.  This thread opens a single OpenGL window using
                the glfw cross-platform library.  It goes into a main loop where
                it polls user events and renders particle data as received.
                - heartbeat thread.  This thread sends a UDP packet to the
                particle server at regular intervals.  The server then
                knows whether and where to stream particle data.  If the
                particle server receives no heartbeat for some period of time,
                it will cease sending packets.  Also, the heartbeat may contain
                simple request information to the server, for example telling
                the server which particle type to send.
                - data thread.  This thread listens for incoming UDP packets,
                decodes them, and updates internal data structures.
Usage       : seewaves --help
============================================================================*/
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "GL/glfw.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <assert.h>
#include "ptp.h"

/* External variables */
extern char *optarg;

/* Versioning */
#define VERSION_HIGH 0
#define VERSION_LOW 1

#define DEFAULT_Z_NEAR 0.1
#define CAMERA_TRANSLATE_SCALER 0.01

typedef enum { HEADS_UP, AXES } seewaves_view_option_t;
typedef enum { SHIFT } seewaves_key_option_t;

/* Global application data structure */
typedef struct _seewaves_t {
	/* verbosity */
	int verbosity;
    /* heartbeat thread, sends packets to server */
    pthread_t heartbeat_thread;
    /* heartbeat thread socket descriptor */
    int heartbeat_socket_fd;
    /* number of heartbeats sent */
    int heartbeats_sent;
    /* data thread, handles incoming data packets */
    pthread_t data_thread;
    /* data thread socket descriptor, incoming data packets */
    int data_socket_fd;
    /* mutex lock for sharing data safely */
    pthread_mutex_t lock;
    /* total number of particles in current simulation */
    unsigned int total_particle_count;
    /* array of x position of all particles, total_particle_cnt long */
    float *x;
    /* array of y position of all particles, total_particle_cnt long */
    float *y;
    /* array of z position of all particles, total_particle_cnt long */
    float *z;
    /* array of w (mass) of all particles, total_particle_cnt long */
    float *w;
    /* eye (camera) position */
    float eye[3];
    /* eye (camera) up vector */
    float up[3];
    /* eye (camera) target position */
    float center[3];
    /* total number of packets received from server */
    int packets_received;
    /* main application loop exit flag */
    int flag_exit_main_loop;
    /* window width */
    int win_width;
    /* window height */
    int win_height;
    /* local server IP to which to bind */
    char data_host[INET6_ADDRSTRLEN];
    /* local server port number to which to bind */
    uint16_t data_port;
    /* remote server host name or IP */
    char gpusph_host[INET6_ADDRSTRLEN];
    /* remote server port number */
    uint16_t gpusph_port;
    /* display red bits */
    int red_bits;
    /* display green bits */
    int green_bits;
    /* display blue bits */
    int blue_bits;
    /* display alpha bits */
    int alpha_bits;
    /* display depth bits */
    int depth_bits;
    /* display stencil bits */
    int stencil_bits;
    /* display mode */
    int display_mode;
    /* background clear color */
    GLfloat background_color[4];
    /* viewport dimensions */
    GLint viewport[4];
    /* most recent timestamp */
    float most_recent_timestamp;
    /* UDP receiver buffer size (optionally set by user) */
    int udp_buffer_size;
    /* view options bit string */
    unsigned char view_options;
    /* z near */
    GLfloat z_near;
    /* z far */
    GLfloat z_far;
    /* mouse x position */
    int mouse_x;
    /* mouse y position */
    int mouse_y;
    /* mouse button id */
    int mouse_button;
    /* mouse button action */
    int mouse_button_action;
    /* mouse wheel position */
    int mouse_wheel_pos;
    /* key options */
    unsigned char key_options;
    /* camera spherical (r) */
    GLfloat camera_r;
    /* camera spherical (theta) */
    GLfloat camera_theta;
    /* camera spherical (fi) */
    GLfloat camera_phi;
} seewaves_t;

/* formatting flag */
typedef enum { BASIC, FULL } seewaves_format_t;

/* Local prototypes */
void *heartbeat_thread_main(void *user_data);
void *data_thread_main(void *user_data);
void camera_reset(void);
int initialize_application(seewaves_t *s, int argc, char **argv);
void initialize_gl(seewaves_t *s);
int display(void);
void GLFWCALL on_key(int key, int action);
void GLFWCALL on_resize(int w, int h);
void util_get_current_time_string(char *buf, ssize_t max_len);
void util_print_seewaves(seewaves_t *s, seewaves_format_t format, int fd);
void camera_set_raw(GLfloat eye_x, GLfloat eye_y, GLfloat eye_z,
		GLfloat up_x, GLfloat up_y, GLfloat up_z,
		GLfloat center_x, GLfloat center_y, GLfloat center_z);

/* Global application data variable */
seewaves_t g_seewaves;

/* Print pthreads error user-defined and internal error message. */
#define PT_ERR_MSG(str, code) { \
		fprintf(stderr, "%s: %s\n", str, strerror(code)); \
}

/*
Print structure information.

@param	s	Application data structure.
@param	format	Format specifier.
@param	fd	File descriptor
*/
void util_print_seewaves(seewaves_t *s, seewaves_format_t format, int fd) {
	FILE *fp = fdopen(fd, "w+");
	if(fp == NULL) {
		perror("util_print_seewaves");
		return;
	}
	fprintf(fp, "verbosity:\t\t%i\n", s->verbosity);
	fprintf(fp, "heartbeats_sent:\t%i\n", s->heartbeats_sent);
	fprintf(fp, "total_particle_count:\t%i\n", s->total_particle_count);
	fprintf(fp, "eye:\t\t\t(%2f, %.2f, %.2f)\n", s->eye[0], s->eye[1], s->eye[2]);
	fprintf(fp, "center:\t\t\t(%2f, %.2f, %.2f)\n", s->center[0], s->center[1], s->center[2]);
	fprintf(fp, "packets_received:\t%i\n", s->packets_received);
	fprintf(fp, "win_width:\t\t%i\n", s->win_width);
	fprintf(fp, "win_height:\t\t%i\n", s->win_height);
	fprintf(fp, "data_host:\t\t%s\n", s->data_host);
	fprintf(fp, "data_port:\t\t%i\n", s->data_port);
	fprintf(fp, "gpusph_host:\t\t%s\n", s->gpusph_host);
	fprintf(fp, "gpusph_port:\t\t%i\n", s->gpusph_port);
	fprintf(fp, "most_recent_timestamp:\t%.2f\n", s->most_recent_timestamp);
	fprintf(fp, "UDP buffer size:\t%i\n", s->udp_buffer_size);
	if(format == FULL) {
		/* dump positions et al, maybe to a file(?) */
	}
}

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
/*
Heartbeat thread loop.
This function is the main loop for the heartbeat thread.  It sends heartbeat
packet to server at regular intervals.

@param  user_data   seewaves_t ptr cast to void ptr.

@returns NULL
*/
void *heartbeat_thread_main(void *user_data) {
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
    ptp_heartbeat_t hb;

    /* current time */
    time_t now;

    /* last time a heartbeat was sent to server */
    time_t last_heartbeat_sent;

    /* loop flag */
    int done = 0;

    /* cast to our global data structure pointer */
    seewaves_t *sw = (seewaves_t*)user_data;

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
        char *ipver;
        if (address_p->ai_family == AF_INET) { /* IPv4 */
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)address_p->ai_addr;
            addr = &(ipv4->sin_addr);
            ipver = "IPv4";
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
            ipver = "IPv6";
        }
    }

    /* free linked-list returned from resolver */
    freeaddrinfo(server_address_info);

    /* clear heartbeat packet */
    memset(&hb, 0, sizeof(ptp_heartbeat_t));

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
                sizeof(ptp_heartbeat_t), 0,
                (const struct sockaddr *)(&heartbeat_socket_remote_address),
                heartbeat_socket_remote_address_len);
            if (bytes_sent == -1) {
                /* failed.  If EBADF, we'll detect that below  */
                if (errno != EBADF) {
                    /* hmm, real failure of some sort */
                    perror("sendto()");
                }
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
            break;
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
                break;
            }
        }
        /* give cpu a break */
        usleep(10);
    }
    /* close our socket */
    close(sw->heartbeat_socket_fd);

    /* we're done */
    return(NULL);
}

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
        if (packet_length_bytes == sizeof(ptp_packet_t)) {
            /* yes, get mutex lock */
            int locked = 0;
            while(!locked) {
                if ((err = pthread_mutex_trylock(&sw->lock)) == 0) {
                    /* got the lock */
                    unsigned int particle = 0;

                    /* exit loop flag */
                    locked++;

                    /* keep most recent timestamp */
                    sw->most_recent_timestamp = packet.t >
                        sw->most_recent_timestamp ? packet.t :
                        sw->most_recent_timestamp;

                    /* keep track of packet count received */
                    sw->packets_received++;
                    /* allocate memory if first time or new particle count */
                    if (sw->total_particle_count !=
                        packet.total_particle_count) {
                        if (sw->x != NULL) {
                            /* not first time, but different count, so free */
                            free(sw->x);
                            free(sw->y);
                            free(sw->z);
                        }
                        sw->x = (float*)calloc(packet.total_particle_count,
                            sizeof(float));
                        sw->y = (float*)calloc(packet.total_particle_count,
                            sizeof(float));
                        sw->z = (float*)calloc(packet.total_particle_count,
                            sizeof(float));
                    }

                    /* save total number of particles in model */
                    sw->total_particle_count = packet.total_particle_count;

                    /* loop through particles in this packet */
                    for(; particle < packet.particle_count; particle++) {
                        /* get particle id */
                        unsigned int id = packet.data[particle].id;

                        /* set x, y, z and w */
                        sw->x[id] = packet.data[particle].position[0];
                        sw->y[id] = packet.data[particle].position[1];
                        sw->z[id] = packet.data[particle].position[2];
                        /*sw->w[id] = packet.data[particle].position[2];*/
                    }

                    /* Release the lock */
                    if ((err = pthread_mutex_unlock(&sw->lock))) {
                        fprintf(stderr, "Error creating mutex: %i\n", err);
                    }
                } else {
                    if(err == EBUSY) {
                        /* mutex was locked */
                    } else if(err == EINVAL) {
                        fprintf(stderr, "Invalid mutex lock?");
                    } else {
                        fprintf(stderr, "Unhandled mutex lock error: %i", err);
                    }
                }
            }
        } else if (packet_length_bytes == 0) {
            /* socket closed on linux */
            break;
        } else {
            if(errno == EINTR) {
                /* ignore */
            } else if (errno == EBADF) {
                /* Parent thread closed the socket, that's our signal that
                we're done */
                done = 1;
            } else {
                perror("recvfrom");
                break;
            }
        }
    }

    /* close our socket */
    close(sw->data_socket_fd);

    /* we're done */
    return(NULL);
}

/*
Set the camera position.
*/
void camera_set_raw(GLfloat eye_x, GLfloat eye_y, GLfloat eye_z,
		GLfloat up_x, GLfloat up_y, GLfloat up_z,
		GLfloat center_x, GLfloat center_y, GLfloat center_z) {
    g_seewaves.eye[0] = eye_x;
    g_seewaves.eye[1] = eye_y;
    g_seewaves.eye[2] = eye_z;
    g_seewaves.up[0] = up_x;
    g_seewaves.up[1] = up_y;
    g_seewaves.up[2] = up_z;
    g_seewaves.center[0] = center_x;
    g_seewaves.center[1] = center_y;
    g_seewaves.center[2] = center_z;
}

/*
Reset the camera position.
*/
void camera_reset(void) {
	camera_set_raw(0.0, 0.1, 1.0, 0.0, 1.0, 0.0, 0.8, 0.4, 0.0);
}

void camera_pan(int units_x, int units_y) {

}

void camera_dolly(int units) {
	/* scale the requested units */
	GLfloat scaled_units = units * CAMERA_TRANSLATE_SCALER;

	/* vector magnitude */
	GLfloat magnitude;

	/* find vector from eye to center */
	GLfloat dir_x = g_seewaves.center[0] - g_seewaves.eye[0];
	GLfloat dir_y = g_seewaves.center[1] - g_seewaves.eye[1];
	GLfloat dir_z = g_seewaves.center[2] - g_seewaves.eye[2];

	/* find magnitude of the dolly move */
	magnitude = sqrt((dir_x * dir_x) + (dir_y * dir_y) + (dir_z * dir_z));

	/* make unit vector */
	dir_x = dir_x / magnitude;
	dir_y = dir_y / magnitude;
	dir_y = dir_y / magnitude;

	g_seewaves.eye[0] = g_seewaves.eye[0] + scaled_units * dir_x;
	g_seewaves.eye[1] = g_seewaves.eye[1] + scaled_units * dir_y;
	g_seewaves.eye[2] = g_seewaves.eye[2] + scaled_units * dir_z;

	g_seewaves.center[0] = g_seewaves.center[0] + scaled_units * dir_x;
	g_seewaves.center[1] = g_seewaves.center[1] + scaled_units * dir_y;
	g_seewaves.center[2] = g_seewaves.center[2] + scaled_units * dir_z;
}

/*
Initialize application internals.

Returns 0 on success, non-zero on failure.
*/
int initialize_application(seewaves_t *s, int argc, char **argv) {
    /* return value */
    int err;

    /* command-line processing counter */
    int option_index;

    /* command-line option return */
    int opt;

    /* command-line options */
    static struct option long_options[] = {
        {"help", no_argument, 0,  'x' },
        {"host", required_argument, 0,  'h' },
        {"port", required_argument, 0,  'p' },
        {"in_host", required_argument, 0,  't' },
        {"in_port", required_argument, 0,  'l' },
        {"verbosity", required_argument, 0,  'v' },
        { 0, 0, 0, 0}
    };

    /* clear the structure */
    memset(&g_seewaves, 0, sizeof(seewaves_t));

    /* set window to default values */
    s->win_width = 800;
    s->win_height = 600;

    /* default local host, port */
    strcpy(s->data_host, PTP_DEFAULT_CLIENT_HOST);
    s->data_port = PTP_DEFAULT_CLIENT_PORT;

    /* default remote host, port */
    strcpy(s->gpusph_host, PTP_DEFAULT_SERVER_HOST);
    s->gpusph_port = PTP_DEFAULT_SERVER_PORT;

    /* display settings */
    s->red_bits = 8;
    s->green_bits = 8;
    s->blue_bits = 8;
    s->alpha_bits = 0;
    s->depth_bits = 24;
    s->stencil_bits = 0;
    s->display_mode = GLFW_WINDOW;

    /* background color */
    s->background_color[0] = 1.0f;
    s->background_color[1] = 1.0f;
    s->background_color[2] = 1.0f;
    s->background_color[3] = 0.0f;

    g_seewaves.viewport[0] = 0;
    g_seewaves.viewport[1] = 0;
    g_seewaves.viewport[2] = g_seewaves.win_width;
    g_seewaves.viewport[3] = g_seewaves.win_height;
    
    g_seewaves.view_options |= 1 << HEADS_UP;
    g_seewaves.view_options |= 1 << AXES;

    g_seewaves.camera_r = 0.000001;

    /* process command-line arguments */
    while ((opt = getopt_long(argc, argv, "h:p:t:r:u:v:", long_options,
        &option_index)) != -1) {
        switch(opt) {
            case 'h':
                strncpy(s->gpusph_host, optarg, INET6_ADDRSTRLEN);
                break;
            case 'p':
                s->gpusph_port = atoi(optarg);
                break;
            case 't':
                strncpy(s->data_host, optarg, INET6_ADDRSTRLEN);
                break;
            case 'r':
                s->data_port = atoi(optarg);
                break;
            case 'u':
                s->udp_buffer_size = atoi(optarg);
                break;
            case 'v':
                s->verbosity = 9;
                break;
            default: {
               	char b[64];
               	util_get_current_time_string(b, sizeof(b));
                printf("seewaves %i.%i (%s)\n\n", VERSION_HIGH, VERSION_LOW, b);
                printf("usage: seewaves [ options ]\n\n");
                printf("Options:\n\n");
                printf("--host -h <address>    GPUSPH host (%s)\n", PTP_DEFAULT_SERVER_HOST);
                printf("--port -p <port>       GPUSPH port (%i)\n", PTP_DEFAULT_SERVER_PORT);
                printf("--in_host -t <address> Incoming host (ALL)\n");
                printf("--in_port -r <port>    Incoming port (%i)\n", PTP_DEFAULT_CLIENT_PORT);
                printf("--udp_size -u <size>   UDP receive buffer size (%i)\n",
                		util_get_udp_buffer_size(-1));
                printf("--verbosity -v <level> Verbosity level 0-9 (0)\n");
                return(-5);
                break;
            }
        }
    }

    /* initialize our mutex lock used to safely update data */
    if ((err = pthread_mutex_init(&s->lock, NULL))) {
        PT_ERR_MSG("pthread_mutex_init", err);
        return(-1);
    }

    /* create data thread */
    if ((err = pthread_create(&s->data_thread, NULL, data_thread_main,
                                (void*)s))) {
        PT_ERR_MSG("data pthread_create", err);
        return(-2);
    }

    /* create heartbeat thread */
    if ((err = pthread_create(&s->heartbeat_thread, NULL, heartbeat_thread_main,
                                (void*)s))) {
        PT_ERR_MSG("heartbeat pthread_create", err);
        return(-3);
    }

    /* reset the camera */
    camera_reset();

    return(0);
}

/*
Perform opengl initialization.

@param s    seewaves pointer
*/
void initialize_gl(seewaves_t *s) {
    /* set background color */
    glClearColor(
        s->background_color[0],
        s->background_color[1],
        s->background_color[2],
        s->background_color[3]);

    /* depth test */
    glEnable(GL_DEPTH_TEST);

    /* smooth */
    glShadeModel(GL_SMOOTH);
}

/*
Render a string at given position.
Leaves matrix unchanged.

@param	x	X units from current raster position
@param	y	Y units from current raster position
@param	z	Z units from current raster position
@param	s	String to render
*/
void render_string(GLfloat x, GLfloat y, GLfloat z, char *s) {
	/* string length */
	int len;

	/* iterator */
	int i;

	/* sanity check */
	assert((s) && (*s));

	/* save matrix state */
	glPushMatrix();

	/* set starting position */
	glRasterPos3f(x, y, z);

	/* get string length */
	len = (int) strlen(s);

	/* loop through all characters */
	for (i = 0; i < len; i++) {
		/* render bitmap character */
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s[i]);
	}

	/* restart matrix state */
	glPopMatrix();
}

/*
Save current projection and modelview matrices on the stack, push standard orthographic.
*/
void push_ortho(void) {
	/* save current projection */
	glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    /* clear current projection */
    glLoadIdentity();
	/* switch to orthographic projection */
    glOrtho(g_seewaves.viewport[0], g_seewaves.viewport[2],
    		g_seewaves.viewport[1] , g_seewaves.viewport[3], -1, 1);
    /* save current modelview */
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    /* clear current modelview */
    glLoadIdentity();
}

/*
Pop previously saved projection and modelview matrices off the stack.
Should have been pushed on with push_ortho().
*/
void pop_ortho(void) {
	/* restore saved projection */
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	/* restore saved modelview */
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

/*
Called from main loop to render the scene.

@returns 1 if redrawn, 0 if unchanged
*/
int display(void) {
    /* return value */
    int err;
    /* loop iterator */
    unsigned int i;

    /* try to lock shared data */
    if ((err = pthread_mutex_trylock(&g_seewaves.lock))) {
    	/* already locked */
        if(err != EBUSY) {
            perror("Error locking mutex during display");
        }
        return(0);
    }

    /* clear the frame */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* reset modelview matrix */
    glLoadIdentity();

    /* aim the camera */
    gluLookAt(
        g_seewaves.eye[0],
        g_seewaves.eye[1],
        g_seewaves.eye[2],
        g_seewaves.center[0],
        g_seewaves.center[1],
        g_seewaves.center[2],
        g_seewaves.up[0],
        g_seewaves.up[1],
        g_seewaves.up[2]);

    glPointSize(1.0f);

    push_ortho();
    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 0.0);
    glVertex2i(g_seewaves.mouse_x, g_seewaves.viewport[3] - g_seewaves.mouse_y);
    glVertex2i(g_seewaves.mouse_x+10, g_seewaves.viewport[3] - g_seewaves.mouse_y);
    glEnd();
    pop_ortho();

    /* does user want axes displayed? */
    if (g_seewaves.view_options & (1 << AXES)) {
		/* we display negative axes slightly darker */
		GLfloat dark_value = 0.6;
		glLineWidth(3.0);
    	glBegin(GL_LINES);
    		/* x axis */
    		glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(1000.0, 0.0, 0.0);
			glColor3f(dark_value, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(-1000.0, 0.0, 0.0);

			/* y axis (rotated to match model) */
			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 1000.0, 0.0);
			glColor3f(0.0, 0.0, dark_value);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, -1000.0, 0.0);

			/* z axis (rotated to match model) */
			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 1000.0);
			glColor3f(0.0, dark_value, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, -1000.0);

		glEnd();
		glLineWidth(1.0);
		glPushMatrix();
		glColor3f(0.0, 0.0, 0.0);
		glTranslatef(g_seewaves.center[0], g_seewaves.center[1], g_seewaves.center[2]);
		glutWireSphere(0.1, 10.0, 10.0);
		glPopMatrix();
    }

    /* draw particles */
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_POINTS);
    for(i = 0; i < g_seewaves.total_particle_count; i++) {
        glVertex3f(g_seewaves.x[i], g_seewaves.z[i], g_seewaves.y[i]);
    }
    glEnd();

    /* is heads up display enabled? */
    if (g_seewaves.view_options & (1 << HEADS_UP)) {
        /* status message buffer */
        char status_msg[1024];
        GLfloat gray = 0.5;
        GLfloat y_inc = 20.0f;
        GLfloat y = 10.0f;
        GLfloat x = 10.0f;

        /* set font color */
        glColor3f(gray, gray, gray);

    	/* switch to ortho mode */
    	push_ortho();

    	/* render network status */
    	sprintf(status_msg, "network: outgoing(%s:%i:%i) incoming(%s:%i:%i)",
    			g_seewaves.gpusph_host, g_seewaves.gpusph_port,
    			g_seewaves.heartbeats_sent,
    			g_seewaves.data_host, g_seewaves.data_port,
    			g_seewaves.packets_received);
    	render_string(x, y, 0.5f, status_msg);
    	y += y_inc;

    	/* render model status */
    	sprintf(status_msg, "model: particles(%i) time(%.3fs)",
    			g_seewaves.total_particle_count,
    			g_seewaves.most_recent_timestamp);
    	render_string(x, y, 0.5f, status_msg);
    	y += y_inc;

    	/* render camera status (rotated to match model) */
    	sprintf(status_msg, "camera: eye(%.2f, %.2f, %.2f) center(%.2f, %.2f, %.2f) up(%.2f, %.2f, %.2f)",
    			g_seewaves.eye[0], g_seewaves.eye[2], g_seewaves.eye[1],
    			g_seewaves.center[0], g_seewaves.center[2], g_seewaves.center[1],
    			g_seewaves.up[0], g_seewaves.up[2], g_seewaves.up[1]);
    	render_string(x, y, 0.5f, status_msg);

    	/* switch back */
    	pop_ortho();
    }
    /* unlock data */
    if ((err = pthread_mutex_unlock(&g_seewaves.lock))) {
        fprintf(stderr, "Error unlocking mutex: %i\n", err);
    }

    /* check for OpenGL errors*/
    while((err = glGetError()) != GL_NO_ERROR) {
    	fprintf(stderr, "OpenGL error: %s\n", gluErrorString(err));
    }

    return(1);
}

void opengl_pos_from_mouse_pos(int mx, int my, GLdouble *x, GLdouble *y, GLdouble *z) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)mx;
    winY = (float)viewport[3] - (float)my;
    glReadPixels(mx, (int)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
    gluUnProject( winX, winY, winZ, modelview, projection, viewport, x, y, z);
}

void GLFWCALL on_mouse(int x, int y) {
	/* check for user pan request */
	if((g_seewaves.mouse_button == GLFW_MOUSE_BUTTON_LEFT) &&
			(g_seewaves.mouse_button_action == GLFW_PRESS) &&
			(g_seewaves.key_options & (1 << SHIFT))) {
		/* calculate difference in mouse position */
		int diff_x = g_seewaves.mouse_x - x;
		int diff_y = g_seewaves.mouse_y - y;
		/* if any difference, pan */
		if((diff_x != 0) || (diff_y != 0)) {
			printf("pressed, panning ");
			camera_pan(diff_x, diff_y);
		}
	} else {
		GLdouble ogl_x, ogl_y, ogl_z;
		opengl_pos_from_mouse_pos(x, y, &ogl_x, &ogl_y, &ogl_z);
		/*printf("%.2f, %.2f, %.2f\n", ogl_x, ogl_y, ogl_z);*/
		/*g_seewaves.center[0] = ogl_x;
		g_seewaves.center[1] = ogl_y;
		g_seewaves.center[2] = ogl_z;*/
		g_seewaves.mouse_x = x;
		g_seewaves.mouse_y = y;
	}
	g_seewaves.mouse_x = x;
	g_seewaves.mouse_y = y;
}

void GLFWCALL on_mouse_button(int button, int action) {
	g_seewaves.mouse_button = button;
	g_seewaves.mouse_button_action = action;
}

void GLFWCALL on_mouse_wheel(int pos) {
	/* calculate difference from previous mouse wheel setting */
	int diff = g_seewaves.mouse_wheel_pos - pos;
	if(diff != 0) {
		camera_dolly(diff);
		g_seewaves.mouse_wheel_pos = pos;
	}
}

/*
Called when a key is pressed or released.

@param	key	Key code
@param	action	Either GLFW_RELEASE or GLFW_PRESS
*/
void GLFWCALL on_key(int key, int action) {
    switch(key) {
        case GLFW_KEY_ESC:
            g_seewaves.flag_exit_main_loop = 1;
            break;
        case GLFW_KEY_LSHIFT:
        case GLFW_KEY_RSHIFT:
        	if(action == GLFW_PRESS) {
        		g_seewaves.key_options |= 1 << SHIFT;
        	} else {
        		g_seewaves.key_options &= ~(1 << SHIFT);
        	}
        	break;
        default:
            /* unhandled key */
            break;
    }
}

void GLFWCALL on_char(int key, int action) {
    switch(key) {
        case GLFW_KEY_ESC:
        case 'q':
            g_seewaves.flag_exit_main_loop = 1;
            break;
        case 'h': {
        	/* toggle heads-up display */
        	g_seewaves.view_options ^= 1 << HEADS_UP;
        	break;
        }
        case 'd': {
        	/* dump internals for inspection */
        	char b[64];
        	util_get_current_time_string(b, sizeof(b));
        	fprintf(stdout, "========= %s =========\n", b);
        	util_print_seewaves(&g_seewaves, FULL, 0);
        	break;
        }
        case 'X': {
        	/* look along X axis */
        	camera_set_raw(5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        	break;
        }
        case 'Y': {
        	/* look along Y axis */
        	camera_set_raw(0.0, 0.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        	break;
        }
        case 'Z': {
        	/* look along Z axis */
        	camera_set_raw(5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        	break;
        }
        case '0': {
        	/* reset camera */
        	camera_reset();
        	break;
        }
        case 'c': {
        	/* cycle through camera/view modes */
        }
        case 'a': {
        	g_seewaves.view_options ^= 1 << AXES;
        	break;
        }
        default:
            /* unhandled key */
            break;
    }
}

void on_resize(int w, int h) {
	/* setup our "camera" */
    glViewport(0, 0, w, h);
    g_seewaves.viewport[0] = 0;
    g_seewaves.viewport[1] = 0;
    g_seewaves.viewport[2] = w;
    g_seewaves.viewport[3] = h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    g_seewaves.z_near = DEFAULT_Z_NEAR;
    g_seewaves.z_far = 10000.0 * g_seewaves.z_near;
    gluPerspective(82.5, (GLfloat)w / (GLfloat)h, g_seewaves.z_near, g_seewaves.z_far);

    /* prepare for modeling and viewing transforms */
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv) {
    /* return value */
    int err;

    /* initialize the application */
    if ((err = initialize_application(&g_seewaves, argc, argv))) {
        exit(EXIT_FAILURE);
    }

    /* we use glut just fonts right now */
    glutInit(&argc, argv);

    /* initialize glfw */
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    /* open the GL window */
    if (!glfwOpenWindow(g_seewaves.win_width,
        g_seewaves.win_height,
        g_seewaves.red_bits,
        g_seewaves.green_bits,
        g_seewaves.blue_bits,
        g_seewaves.alpha_bits,
        g_seewaves.depth_bits,
        g_seewaves.stencil_bits,
        g_seewaves.display_mode)) {
        return(-1);
    }

    /* set a keyboard callback function */
    glfwSetCharCallback(on_char);
    glfwSetKeyCallback(on_key);

    /* set a mouse callback */
    glfwSetMousePosCallback(on_mouse);

    /* set a mouse button callback */
    glfwSetMouseButtonCallback(on_mouse_button);

    /* set a mouse wheel callback */
    glfwSetMouseWheelCallback(on_mouse_wheel);

    /* set a window resize callback function */
    glfwSetWindowSizeCallback(on_resize);

    /* perform any gl-related initialization */
    initialize_gl(&g_seewaves);

    /* optionally dump some internals */
    if(g_seewaves.verbosity) {
    	util_print_seewaves(&g_seewaves, FULL, 0);
    }

    /* loop until the user closes the window or exits */
    while (g_seewaves.flag_exit_main_loop != 1) {
        /* did user close window? */
        if (!glfwGetWindowParam(GLFW_OPENED)) {
            /* yes, gracefully exit */
            g_seewaves.flag_exit_main_loop = 1;
        }

        /* render display */
        if(display()) {
            /* swap the display buffer */
            glfwSwapBuffers();
        }

        /* give the CPU a break */
        usleep(10);
    }

    /* close the underlying sockets, results in threads exiting gracefully */
#ifdef __APPLE__
    close(g_seewaves.data_socket_fd);
    close(g_seewaves.heartbeat_socket_fd);
#else
    shutdown(g_seewaves.data_socket_fd, SHUT_RDWR);
    shutdown(g_seewaves.heartbeat_socket_fd, SHUT_RDWR);
#endif

    /* wait for threads to finish */
    if ((err = pthread_join(g_seewaves.data_thread, NULL))) {
        PT_ERR_MSG("pthread_join(data_thread)", err);
    }
    if ((err = pthread_join(g_seewaves.heartbeat_thread, NULL))) {
        PT_ERR_MSG("pthread_join(heartbeat_thread)", err);
    }

    /* terminate glfw */
    glfwTerminate();

    exit(EXIT_SUCCESS);
}
