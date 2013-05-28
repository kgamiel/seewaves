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
Usage       : seewaves [ server port ]
                server - name or IP address of particle server (GPUSPH),
                defaults to 127.0.0.1
                port - port number for particle server, defaults to 50001
============================================================================*/
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <GL/glfw.h>
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
#include "ptp.h"

/* External variables */
extern char *optarg;

/* Versioning */
#define VERSION_HIGH 0
#define VERSION_LOW 1

/* Global application data structure */
typedef struct _seewaves_t {
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
    /* udp buffer length in use */
    int udp_buf_size;
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
} seewaves_t;

/* Local prototypes */
void *heartbeat_thread_main(void *user_data);
void *data_thread_main(void *user_data);
void camera_reset(void);
int initialize_application(seewaves_t *s, int argc, char **argv);
void initialize_gl(seewaves_t *s);
void display(void);
void GLFWCALL on_key(int key, int action);
void GLFWCALL on_resize(int w, int h);
void view_ortho(int x, int y);
void display_status(char *s);
void view_perspective(void);

/* Global application data variable */
seewaves_t g_seewaves;

/* Print pthreads error user-defined and internal error message. */
#define PT_ERR_MSG(str, code) { \
		fprintf(stderr, "%s: %s\n", str, strerror(code)); \
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
    int err;

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
        fprintf(stderr, "getaddrinfo() failed: %s", gai_strerror(err));
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
                sizeof(address_p->ai_addr));
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
    time(&last_heartbeat_sent);

    /* main thread loop */
    while(!done) {
        /* buffer used to test for socket closure */
        unsigned char b[64];

        /* get current time */
        time(&now);

        /* is it time to send a heartbeat? */
        if (difftime(now, last_heartbeat_sent) > PTP_HEARTBEAT_TTL_S) {
            /* yes, send a heartbeat */
            int bytes_sent;
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
        if (err <= 0) {
            done = 1;
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
    struct addrinfo *res, *p;

    /* exponent */
    double exp = 1;

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
    if((err=getaddrinfo(sw->data_host, port_as_string, &hints, &res))) {
       fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(err));
       exit(1);
    }

    /* walk through resulting list of addresses, use first IPv4 */
    for(p = res; p!=NULL;p = p->ai_next) {
        if(p->ai_family == AF_INET) {
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ai_addr;
            if(inet_ntop(res->ai_family, &(ipv4->sin_addr),
                sw->data_host, sizeof(sw->data_host)) == NULL) {
                perror("inet_ntop error");
                exit(1);
            }
            break;
        }
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

    /* Set maximum UDP receiver buffer size.  Strategy is to increase until
    it fails, then fall back to last one that worked (must be a better way!) */
#define SET_RECV_BUF
#ifdef SET_RECV_BUF
    for(;; exp++) {
        /* compute a value */
        int a;
        a = sizeof(ptp_packet_t) * (int)pow(2.0, exp);

        /* try to set to that value (up to 1024 times packet size) */
        if ((a > (1024 * (int)sizeof(ptp_packet_t))) ||
            (setsockopt(sw->data_socket_fd, SOL_SOCKET, SO_RCVBUF, &a,
            (socklen_t)(sizeof(int))) == -1)) {
            /* Intentional failure, set it to highest successful value */
            a = sizeof(ptp_packet_t) * pow(2, (exp - 1));
            if (setsockopt(sw->data_socket_fd, SOL_SOCKET, SO_RCVBUF, &a,
                           sizeof(int)) == -1) {

                /* Unintentional failure, uh oh */
                perror("setsockopt(SO_RCVBUF)");
            }
            break;
        }
    }
#endif
    /* get actual UDP receive buffer size in use */
    if (getsockopt(sw->data_socket_fd, SOL_SOCKET, SO_RCVBUF, &sw->udp_buf_size,
                  (socklen_t*)&err) == -1) {
        perror("getsocketbufsize");
    }

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
                if (pthread_mutex_trylock(&sw->lock)) {
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
Reset the camera position.
*/
void camera_reset(void) {
    g_seewaves.eye[0] = 1.5;
    g_seewaves.eye[1] = 1.0;
    g_seewaves.eye[2] = 0.3;
    g_seewaves.up[0] = 0.0;
    g_seewaves.up[1] = 0.0;
    g_seewaves.up[2] = 1.0;
    g_seewaves.center[0] = 1.0;
    g_seewaves.center[1] = 0.0;
    g_seewaves.center[2] = 0.0;
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
        {"local_host", required_argument, 0,  't' },
        {"local_port", required_argument, 0,  'l' },
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

    /* process command-line arguments */
    while ((opt = getopt_long(argc, argv, "h:p:t:r:", long_options,
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
            default:
                printf("seewaves %i.%i\n\n", VERSION_HIGH, VERSION_LOW);
                printf("usage: seewaves [ options ]\n\n");
                printf("Options:\n\n");
                printf("--host -h <server>     GPUSPH host (127.0.0.1)\n");
                printf("--port -p <port>       GPUSPH port (5001)\n");
                printf("--in_host -t <server>  Incoming host (127.0.0.1)\n");
                printf("--in_port -r <port>    Incoming port (5000)\n");
                return(-5);
                break;
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

void view_perspective(void) {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

/* heads up display */
void view_ortho(int x, int y) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, x , 0, y , -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
}

void display_status(char *s) {
    int len, i;
    view_ortho(g_seewaves.viewport[2], g_seewaves.viewport[3]); 
    /*float3 tank_size = problem->m_size;*/
    glRasterPos2i(10, 10);
    /*glRasterPos3f(tank_size.x, tank_size.y, 0.0);*/
    len = (int) strlen(s);
    for (i = 0; i < len; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s[i]);
    }
    view_perspective();
}

/*
Called at the start of the program, after a glutPostRedisplay() and during idle
to display a frame
*/
void display(void) {
    /* return value */
    int err;
    /* loop iterator */
    unsigned int i;

    /* status message buffer */
    char status_msg[1024];

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glPointSize(3.0f);
    glLineWidth(1.0f);

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

    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1000.0, 0.0, 0.0);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1000.0, 0.0);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1000.0);
    glEnd();
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_POINTS);
    if ((err = pthread_mutex_lock(&g_seewaves.lock))) {
        fprintf(stderr, "Error locking mutex: %i\n", err);
    }
    for(i = 0; i < g_seewaves.total_particle_count; i++) {
        glVertex3f(g_seewaves.x[i], g_seewaves.y[i], g_seewaves.z[i]);
    }
    if ((err = pthread_mutex_unlock(&g_seewaves.lock))) {
        fprintf(stderr, "Error unlocking mutex: %i\n", err);
    }
    glEnd();

    /* display status */
    sprintf(status_msg,
        "hb=%s:%i, data=%s:%i,%i packets, %i particles, t=%.3fs",
        g_seewaves.gpusph_host, g_seewaves.gpusph_port,
        g_seewaves.data_host, g_seewaves.data_port,
        g_seewaves.packets_received, 
        g_seewaves.total_particle_count,
        g_seewaves.most_recent_timestamp);
    display_status(status_msg);
}

void GLFWCALL on_key(int key, int action) {
    switch(key) {
        case GLFW_KEY_ESC:
            if(action == GLFW_RELEASE) {
                g_seewaves.flag_exit_main_loop = 1;
            }
            break;
        default:
            /* unhandled key */
            break;
    }
}

void on_resize(int w, int h) {
    glViewport(0, 0, w, h);
    g_seewaves.viewport[0] = 0;
    g_seewaves.viewport[1] = 0;
    g_seewaves.viewport[2] = w;
    g_seewaves.viewport[3] = h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(100.0, (GLfloat)w / (GLfloat)h, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv) {
    /* return value */
    int err;

    /* we use glut just fonts right now */
    glutInit(&argc, argv);

    /* initialize the application */
    if ((err = initialize_application(&g_seewaves, argc, argv))) {
        exit(err);
    }

    /* initialize glfw */
    if (!glfwInit()) {
        exit(-1);
    }

    g_seewaves.viewport[0] = 0;
    g_seewaves.viewport[1] = 0;
    g_seewaves.viewport[2] = g_seewaves.win_width;
    g_seewaves.viewport[3] = g_seewaves.win_height;

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
    glfwSetKeyCallback(on_key);

    /* set a window resize callback function */
    glfwSetWindowSizeCallback(on_resize);

    /* perform any gl-related initialization */
    initialize_gl(&g_seewaves);

    /* loop until the user closes the window or exits */
    while (g_seewaves.flag_exit_main_loop != 1) {
        /* did user close window? */
        if (!glfwGetWindowParam(GLFW_OPENED)) {
            /* yes, gracefully exit */
            g_seewaves.flag_exit_main_loop = 1;
        }

        /* render display */
        display();

        /* swap the display buffer */
        glfwSwapBuffers();

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

    return(0);
}
