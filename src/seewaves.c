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
#include "heartbeat.h"
#include "data_thread.h"
#include "ptp.h"
#include "cfg.h"
#include "ArcBall.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "util.h"
#include "seewaves.h"

// Uncomment for experimental mutex locking, not sure it's needed
//#define LOCK 1

/* External variables */
extern char *optarg;

/* Local prototypes */
void cfg_print(FILE *fp);
void physics_update(long usec);
void camera_reset(void);
int initialize_application(seewaves_t *s, int argc, char **argv);
void initialize_gl(seewaves_t *s);
int display(void);
void GLFWCALL on_key(int key, int action);
void GLFWCALL on_resize(int w, int h);
void camera_set_raw(GLfloat eye_x, GLfloat eye_y, GLfloat eye_z,
		GLfloat up_x, GLfloat up_y, GLfloat up_z,
		GLfloat target_x, GLfloat target_y, GLfloat target_z);
void camera_dolly(int units);
void render_string(GLfloat x, GLfloat y, GLfloat z, char *s);
void render_grid_sub(float extent, float space);
void push_ortho(void);
void pop_ortho(void);
void render_fading_text(GLfloat x, GLfloat y, GLfloat z, char *s, GLfloat t);
void render_grid(GLfloat extent);
void opengl_pos_from_mouse_pos(int mx, int my, GLdouble *x, GLdouble *y,
    GLdouble *z);
void GLFWCALL on_mouse(int x, int y);
void GLFWCALL on_mouse_button(int button, int action);
void GLFWCALL on_mouse_wheel(int pos);
void GLFWCALL on_char(int key, int action);
int get_int(char *name);
float get_float(char *name);
char *get_string(char *name);
int application_reconfigure(seewaves_t *s, const char *dirname,
    const char *filename, int create);
float *get_float3(char *name, float *value);
void set_float3(char *name, float x, float y, float z);
const char *byte_to_binary(int x);
void render_axes(float x, float y, float z, float length);
void render_box(float origin[3], float size[3]);

/* Global application data variable */
static seewaves_t g_seewaves;

/* Print pthreads error user-defined and internal error message. */
#define PT_ERR_MSG(str, code) { \
		fprintf(stderr, "%s: %s\n", str, strerror(code)); \
}

/* Configuration options */
cfg_option_t g_config_options[] = {
		{ CFG_WIN_TITLE, "Main window title",      STRING,  { ""      }, { "Seewaves" } },
		{ CFG_WIN_X,     "Main window X position", INTEGER, { .ival=0 }, { .ival=100  } },
		{ CFG_WIN_Y,     "Main window Y position", INTEGER, { .ival=0 }, { .ival=100  } },
		{ CFG_WIN_WIDTH, "Main window width",      INTEGER, { .ival=0 }, { .ival=800  } },
		{ CFG_WIN_HEIGHT,"Main window height",     INTEGER, { .ival=0 }, { .ival=600  } },
		{ CFG_EYE_POS,   "Eye position",           FLOAT3,  { .ival=0 }, { .f3val = { 1.0, 1.0, 1.0 } } },
		{ CFG_EYE_UP,    "Eye up vector",          FLOAT3,  { .ival=0 }, { .f3val = { 0.0, 0.0, 1.0 } } },
		{ CFG_EYE_TARGET,"Eye target position",    FLOAT3,  { .ival=0 }, { .f3val = { 0.0, 0.0, 0.0 } } },
		{ CFG_ZNEAR,     "Z near",                 FLOAT,   { .fval=0 }, { .fval=0.1  } },
		{ CFG_ZFAR,      "Z far",                  FLOAT,   { .fval=0 }, { .fval=10000.0  } },
		{ CFG_FLUID_COLOR,"Fluid color",           FLOAT3,  { .ival=0 }, { .f3val = { 0.0, 0.0, 1.0 } } },
		{ CFG_BOUNDARY_COLOR,"Boundary color",     FLOAT3,  { .ival=0 }, { .f3val = { 0.0, 0.0, 0.0 } } },
		{ CFG_PISTON_COLOR,"Piston color",         FLOAT3,  { .ival=0 }, { .f3val = { 1.0, 0.0, 0.0 } } },
		{ CFG_PADDLE_COLOR,"Paddle color",         FLOAT3,  { .ival=0 }, { .f3val = { 0.2, 0.2, 0.2 } } },
		{ CFG_GATE_COLOR,"Gate color",             FLOAT3,  { .ival=0 }, { .f3val = { 1.0, 1.0, 0.0 } } },
		{ CFG_OBJECT_COLOR,"Object color",         FLOAT3,  { .ival=0 }, { .f3val = { 0.0, 0.0, 0.0 } } },
		{ CFG_TESTPOINT_COLOR,"Test point color",  FLOAT3,  { .ival=0 }, { .f3val = { 1.0, 0.0, 0.0 } } },
		{ CFG_SURFACE_COLOR,"Surface color",       FLOAT3,  { .ival=0 }, { .f3val = { 1.0, 0.0, 0.0 } } },
		{ NULL,          NULL,                     0,       { ""      }, { NULL       } }
};

void cfg_print(FILE *fp) {
	cfg_option_t *option;
	int i;
	for( i = 0; ; i++) {
		option = &(g_config_options[i]);
		if(option->name == NULL) {
			break;
		}
		fprintf(fp, "# %s\n%s ", option->description, option->name);
		switch(option->which) {
		case STRING: {
			fprintf(fp, "%s\n\n", option->d.sval);
			break;
		}
		case INTEGER: {
			fprintf(fp, "%i\n\n", option->d.ival);
			break;
		}
		case FLOAT: {
			fprintf(fp, "%.3f\n\n", option->d.fval);
			break;
		}
		case FLOAT3: {
			fprintf(fp, "%.3f %.3f %.3f\n\n", option->d.f3val[0], option->d.f3val[1], option->d.f3val[2]);
			break;
		}
		}
	}
}

/*
Print structure information.

@param	s	Application data structure.
@param	format	Format specifier.
@param	fd	File descriptor
*/
void util_print_seewaves(seewaves_t *s, seewaves_format_t format, int fd) {
	float fv[3];
	FILE *fp = fdopen(fd, "w+");
	if(fp == NULL) {
		perror("util_print_seewaves");
		return;
	}
	fprintf(fp, "verbosity:\t\t%i\n", s->verbosity);
	fprintf(fp, "heartbeats_sent:\t%i\n", s->heartbeats_sent);
	fprintf(fp, "total_particle_count:\t%i\n", s->total_particle_count);
	get_float3(CFG_EYE_POS, fv);
	fprintf(fp, "eye:\t\t\t(%2f, %.2f, %.2f)\n", fv[0], fv[1], fv[2]);
	get_float3(CFG_EYE_TARGET, fv);
	fprintf(fp, "target:\t\t\t(%2f, %.2f, %.2f)\n", fv[0], fv[1], fv[2]);
	fprintf(fp, "udp_max_packet_size:\t%i\n", PTP_UDP_PACKET_MAX);
	fprintf(fp, "particles_per_packet:\t%ld\n", PTP_PARTICLES_PER_PACKET);
	fprintf(fp, "packet_hdr_size:\t%ld\n", PTP_PACKET_HEADER_SIZE);
	fprintf(fp, "particle_data_size\t%ld\n", sizeof(ptp_particle_data_t));
	fprintf(fp, "packet_size (incs %ld for each of %ld particles):%ld\n", 
        sizeof(ptp_particle_data_t), PTP_PARTICLES_PER_PACKET, sizeof(ptp_packet_t));
	fprintf(fp, "packet_per_udp_buf:\t%ld\n", (s->udp_buffer_size/sizeof(ptp_packet_t)));
	fprintf(fp, "packets_received:\t%i\n", s->packets_received);
	fprintf(fp, "win_width:\t\t%i\n", get_int(CFG_WIN_WIDTH));
	fprintf(fp, "win_height:\t\t%i\n", get_int(CFG_WIN_HEIGHT));
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

const char *byte_to_binary(int x) {
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return(b);
}

/*
Set the camera position.
*/
void camera_set_raw(GLfloat eye_x, GLfloat eye_y, GLfloat eye_z,
		GLfloat up_x, GLfloat up_y, GLfloat up_z,
		GLfloat target_x, GLfloat target_y, GLfloat target_z) {
    set_float3(CFG_EYE_POS, eye_x, eye_y, eye_z);
    set_float3(CFG_EYE_UP, up_x, up_y, up_z);
    set_float3(CFG_EYE_TARGET, target_x, target_y, target_z);
}

/*
Reset the camera position.
*/
void camera_reset(void) {
	static int first = 1;
    static float eye[3];
    static float target[3];

    g_seewaves.model_pan[0] = 0.0;
	g_seewaves.model_pan[1] = 0.0;

    if(first) {
    	get_float3(CFG_EYE_POS, eye);
    	get_float3(CFG_EYE_TARGET, target);
        set_float3(CFG_EYE_POS, eye[0], eye[1], eye[2]);
        set_float3(CFG_EYE_TARGET, target[0], target[1], target[2]);
    	first = 0;
    }
	camera_set_raw(eye[0], eye[1], eye[2], 0.0, 1.0, 0.0, target[0], target[1], target[2]);
}

void camera_dolly(int units) {
    float x, y, z;
    GLfloat dir_x, dir_y, dir_z;
    float eye[3];
    float target[3];

	/* scale the requested units */
	GLfloat scaled_units = units * CAMERA_TRANSLATE_SCALER;

	/* vector magnitude */
	GLfloat magnitude;

	/* find vector from eye to center */
	get_float3(CFG_EYE_POS, eye);
	get_float3(CFG_EYE_TARGET, target);
	dir_x = target[0] - eye[0];
	dir_y = target[1] - eye[1];
	dir_z = target[2] - eye[2];

	/* find magnitude of the dolly move */
	magnitude = sqrt((dir_x * dir_x) + (dir_y * dir_y) + (dir_z * dir_z));

	/* make unit vector */
	dir_x = dir_x / magnitude;
	dir_y = dir_y / magnitude;
	dir_z = dir_z / magnitude;

	x = eye[0] + scaled_units * dir_x;
	y = eye[1] + scaled_units * dir_y;
	z = eye[2] + scaled_units * dir_z;

    set_float3(CFG_EYE_POS, x, y, z);
}

char *get_string(char *name) {
	cfg_option_t *option = cfg_get(&g_seewaves.config, name);
	if(option && option->which == STRING) {
		return(option->u.sval);
	}
	return NULL;
}

int get_int(char *name) {
	cfg_option_t *option = cfg_get(&g_seewaves.config, name);
	if(option && option->which == INTEGER) {
		return(option->u.ival);
	}
	return(0);
}

float get_float(char *name) {
	cfg_option_t *option = cfg_get(&g_seewaves.config, name);
	if(option && option->which == FLOAT) {
		return(option->u.fval);
	}
	return(0.0);
}

float *get_float3(char *name, float *value) {
	cfg_option_t *option = cfg_get(&g_seewaves.config, name);
	if(option && option->which == FLOAT3) {
		value[0] = option->u.f3val[0];
		value[1] = option->u.f3val[1];
		value[2] = option->u.f3val[2];
		return(value);
	}
	return(NULL);
}

void set_float3(char *name, float x, float y, float z) {
	cfg_option_t *option = cfg_get(&g_seewaves.config, name);
	if(option && option->which == FLOAT3) {
		option->u.f3val[0] = x;
		option->u.f3val[1] = y;
		option->u.f3val[2] = z;
	}
}

int application_reconfigure(seewaves_t *s, const char *dirname,
    const char *filename, int create) {
	char path[FILENAME_MAX];
	cfg_option_t *option;
	int i;

	/* file status */
	struct stat status;

	/* does directory exist? */
	if(stat(dirname, &status)) {
		/* no, does user want it created? */
		if(create) {
			/* yes, create it */
			if(mkdir(dirname, S_IRWXU)) {
				perror(dirname);
				return(0);
			}
		} else {
			/* no, we're done here */
			return(0);
		}
	}
	/* does file exist? */
	sprintf(path, "%s/%s", dirname, filename);
	if(stat(path, &status)) {
		/* no, does user want it created? */
		if(create) {
			/* yes, create it */
			FILE *fp = fopen(path, "w");
			if(fp == NULL) {
				perror(path);
				return(0);
			}
			cfg_print(fp);
			fclose(fp);
		} else {
			/* no, we're done here */
			return(0);
		}
	}

	/* load the configuration file */
	if(cfg_open(&s->config, path, g_config_options) == CFG_ERROR) {
		return(0);
	}

	/* load options */
	for( i = 0; ; i++) {
		option = &(g_config_options[i]);
		if(option->name == NULL) {
			break;
		}
		switch(option->which) {
			case STRING: {
				cfg_get_string(&s->config, option->name, (char*)option->u.sval, MAX_CFG_STRING, option->d.sval);
				break;
			}
			case INTEGER: {
				option->u.ival = cfg_get_int(&s->config, option->name, (int)option->u.ival);
				break;
			}
			case FLOAT: {
				option->u.fval = cfg_get_float(&s->config, option->name, (float)option->u.fval);
				break;
			}
			case FLOAT3: {
				/* convert from string to three floats */
				double x, y, z;
				char buf[MAX_CFG_STRING];
				sprintf(buf, "%.12f %.12f %.12f", option->d.f3val[0], option->d.f3val[1],
						option->d.f3val[2]);
				cfg_get_string(&s->config, option->name, buf, MAX_CFG_STRING, buf);
				if(sscanf(buf, "%lf %lf %lf", &x, &y, &z) == 3) {
					option->u.f3val[0] = x;
					option->u.f3val[1] = y;
					option->u.f3val[2] = z;
				}
				break;
			}
		}
	}

	return(1);
}

/*
Initialize application internals.

Returns 0 on success, non-zero on failure.
*/
int initialize_application(seewaves_t *s, int argc, char **argv) {
	/* directory name buffer */
	char dirname[FILENAME_MAX];

	/* filename buffer */
	char filename[FILENAME_MAX];

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

    g_seewaves.view_options |= 1 << HEADS_UP;
    //g_seewaves.view_options |= 1 << AXES;
    //g_seewaves.view_options |= 1 << GRID;
    //g_seewaves.view_options |= 1 << ROTATION_AXES;

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

    /* optionally override default configuration with globally-defined */
    sprintf(dirname, "%s/.seewaves", getenv("HOME"));
    sprintf(filename, "seewaves.cfg");
    if(application_reconfigure(s, dirname, filename, 1) != 1) {
    	fprintf(stderr, "User configurations disabled\n");
    }

    /* optionally override configuration with locally-defined */
    sprintf(dirname, ".");
    (void)application_reconfigure(s, dirname, filename, 0);

    arcball_init(&s->arcball, get_int(CFG_WIN_WIDTH), get_int(CFG_WIN_HEIGHT));
    Quaternion_loadIdentity(&s->arcball_rotation);
    Matrix_loadIdentity(&s->arcball_transform);
    Matrix_loadIdentity(&s->arcball_last_rotation);
    Matrix_loadIdentity(&s->arcball_this_rotation);

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

    glEnable(GL_POINT_SMOOTH);

    /* enable blending for translucence */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable (GL_LINE_SMOOTH);
    glHint (GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glGetFloatv(GL_SMOOTH_LINE_WIDTH_RANGE, s->line_width_range);
	glGetFloatv(GL_SMOOTH_LINE_WIDTH_GRANULARITY, &s->line_width_step);
	glGetFloatv(GL_POINT_SIZE_RANGE, s->point_size_range);
	glGetFloatv(GL_POINT_SIZE_GRANULARITY, &s->point_size_step);

	/* setup viewport for this rendering */
	glViewport(g_seewaves.viewport_main[0], g_seewaves.viewport_main[1],
    		g_seewaves.viewport_main[2], g_seewaves.viewport_main[3]);

	/* setup projection for this viewport */
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(82.5, g_seewaves.viewport_main[2] /
        g_seewaves.viewport_main[3], get_float(CFG_ZNEAR), get_float(CFG_ZFAR));

    /* prepare for modeling and viewing transforms */
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
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
	glLoadIdentity();

	/* set starting position */
	glRasterPos3f(x, y, z);

	/* get string length */
	len = (int) strlen(s);

	/* loop through all characters */
	for (i = 0; i < len; i++) {
		/* render bitmap character */
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, s[i]);
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
    glOrtho(g_seewaves.viewport_main[0], g_seewaves.viewport_main[2],
    		g_seewaves.viewport_main[1] , g_seewaves.viewport_main[3], -1, 1);

    /* switch to modelview */
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
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

	/* switch to modelview */
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void render_fading_text(GLfloat x, GLfloat y, GLfloat z, char *s, GLfloat t) {
	time_t now;
	time(&now);
	g_seewaves.fade_position[0] = x;
	g_seewaves.fade_position[1] = y;
	g_seewaves.fade_position[2] = z;
	g_seewaves.fade_start = now;
	g_seewaves.fade_text = strdup(s);
	g_seewaves.fade_duration = t;
}

void render_grid_sub(float extent, float space) {
	float threshold = 0.000009;
	float result;
	float x;
	float y;
	int count = (int)(extent/space);
	int i;
	for(i = 0; i < count; i++) {
		x = space * i;
		errno = 0;
		result = fmodf(x, space * 10.0);
		if(errno) {
			perror("fmodf");
			continue;
		}
		if(result < threshold) {
			continue;
		}
		glVertex3f(x, 0.0, 0.0);
		glVertex3f(x, extent, 0.0);
	}
	for(i = 0; i < count; i++) {
		y = space * i;
		errno = 0;
		result = fmodf(y, space * 10.0);
		if(errno) {
			perror("fmodf");
			continue;
		}
		if(result < threshold) {
			continue;
		}
		glVertex3f(0.0, y, 0.0);
		glVertex3f(extent, y, 0.0);
	}
}

void render_grid(GLfloat extent) {
	GLfloat color = 0.9;
	GLfloat grid_large_color = 0.7;
	GLfloat grid_medium_color = 0.8;
	GLfloat grid_small_color = 0.9;
	glColor3f(color, color, color);
	glBegin(GL_LINES);
	/* Draw smaller grid cells */
	glColor3f(grid_small_color, grid_small_color, grid_small_color);
	render_grid_sub(extent, 0.1);
	/* Draw medium grid cells */
	glColor3f(grid_medium_color, grid_medium_color, grid_medium_color);
	render_grid_sub(extent, 1.0);
	/* Draw large grid cells */
	glColor3f(grid_large_color, grid_large_color, grid_large_color);
	render_grid_sub(extent, 10.0);
	glEnd();
}

#define OBJECTPART (6 << 4)

void render_axes(float x, float y, float z, float length) {
	glTranslatef(x, y, z);
	glBegin(GL_LINES);
		/* x axis */
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(length, 0.0, 0.0);

		/* y axis (rotated to match model) */
		glColor3f(0.0, 0.0, 1.0);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(0.0, length, 0.0);

		/* z axis (rotated to match model) */
		glColor3f(0.0, 1.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, length);
	glEnd();
}

void render_box(float origin[3], float size[3]) {
	if((size[0] == 0.0) || (size[1] == 0.0) || (size[2] == 0.0)) {
		return;
	}

	/* move to origin */
	glTranslatef(origin[0], origin[1], origin[2]);

	/* draw back side */
	glColor4f(1.0, 0.0, 0.0, 0.04);
	glRectf(0.0, 0.0, size[0], size[2]);

	glTranslatef(size[0], 0.0, 0.0);
	glRotatef(270.0, 0.0, 1.0, 0.0);
	glRectf(0.0, 0.0, size[1], size[2]);

	glTranslatef(size[1], 0.0, 0.0);
	glRotatef(270.0, 0.0, 1.0, 0.0);
	glRectf(0.0, 0.0, size[0], size[2]);

	glTranslatef(size[0], 0.0, 0.0);
	glRotatef(270.0, 0.0, 1.0, 0.0);
	glRectf(0.0, 0.0, size[1], size[2]);

	glTranslatef(size[1], 0.0, 0.0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glRotatef(90.0, 0.0, 0.0, 1.0);
	glRectf(0.0, 0.0, size[0], size[1]);
}

/*
Called from main loop to render the scene.

@returns 1 if redrawn, 0 if unchanged
*/
int display(void) {
	/* A more full-featured config will solve this hack */
	float fluid_color[3];
	float boundary_color[3];
	float piston_color[3];
	float paddle_color[3];
	float gate_color[3];
	float object_color[3];
	float testpoint_color[3];
	float surface_color[3];

	get_float3(CFG_FLUID_COLOR, fluid_color);
	get_float3(CFG_BOUNDARY_COLOR, boundary_color);
	get_float3(CFG_PISTON_COLOR, piston_color);
	get_float3(CFG_PADDLE_COLOR, paddle_color);
	get_float3(CFG_GATE_COLOR, gate_color);
	get_float3(CFG_OBJECT_COLOR, object_color);
	get_float3(CFG_SURFACE_COLOR, surface_color);

	unsigned int particles_in_current_timestep = 0;

    /* return value */
    int err;

    /* loop iterator */
    unsigned int i;

    /* world extent */
	GLfloat extent = 100;

    /* cache eye for speed */
    GLfloat eye[3];
    GLfloat up[3];
    GLfloat target[3];

	/* setup viewport for this rendering */
	glViewport(g_seewaves.viewport_main[0], g_seewaves.viewport_main[1],
    		g_seewaves.viewport_main[2], g_seewaves.viewport_main[3]);

	/* setup projection for this viewport */
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(82.5, g_seewaves.viewport_main[2] / g_seewaves.viewport_main[3],
    		get_float(CFG_ZNEAR), get_float(CFG_ZFAR));

    /* prepare for modeling and viewing transforms */
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    /* has user requested help screen? */
    /*
    if(g_seewaves.show_help) {
        glColor3f(FONT_GRAY, FONT_GRAY, FONT_GRAY);

    	push_ortho();
    	pop_ortho();
    	return 1;
    }
	*/


    /* try to lock shared data */
#ifdef LOCK
    if ((err = pthread_mutex_trylock(&g_seewaves.lock))) {
    	/* already locked */
        if(err != EBUSY) {
            perror("Error locking mutex during display");
        }
        return(0);
    }
#endif

	glPushMatrix();
	glMultMatrixf(g_seewaves.arcball_transform.m);


    /* aim the camera */
    get_float3(CFG_EYE_POS, eye);
    get_float3(CFG_EYE_UP, up);
    get_float3(CFG_EYE_TARGET, target);
    gluLookAt(
        eye[0],
        eye[1],
        eye[2],
        target[0],
        target[1],
        target[2],
		up[0],
		up[1],
		up[2]);

    glTranslatef(g_seewaves.rotation_center[0],g_seewaves.rotation_center[2],
    		g_seewaves.rotation_center[1]);
    glRotatef(g_seewaves.model_rotation[0], 1.0, 0.0, 0.0);
    glRotatef(g_seewaves.model_rotation[1], 0.0, 1.0, 0.0);
    glTranslatef(g_seewaves.model_pan[1], -g_seewaves.model_pan[0], 0.0);
    glTranslatef(-g_seewaves.rotation_center[0],-g_seewaves.rotation_center[2],
    		-g_seewaves.rotation_center[1]);

    glPointSize(g_seewaves.point_size_range[0]);
	glLineWidth(g_seewaves.line_width_range[0]);

    /* does user want grid displayed? */
    if (g_seewaves.view_options & (1 << GRID)) {
    	glPushMatrix();
    	render_grid(extent);
    	glRotatef(90.0, 1.0, 0.0, 0.0);
    	render_grid(extent);
    	glRotatef(90.0, 0.0, 1.0, 0.0);
    	render_grid(extent);
    	glPopMatrix();
    }

    /* does user want axes displayed? */
    if (g_seewaves.view_options & (1 << AXES)) {
    	glPushMatrix();
    	render_axes(0.0, 0.0, 0.0, extent);
    	glPopMatrix();
    }

    /* does user want rotation center axes displayed? */
    if (g_seewaves.view_options & (1 << ROTATION_AXES)) {
    	glPushMatrix();
    	render_axes(g_seewaves.rotation_center[0], g_seewaves.rotation_center[1],
    			g_seewaves.rotation_center[2], 100.0);
    	glPopMatrix();
    }

    /* draw particles */
    glBegin(GL_POINTS);
    for(i = 0; i < g_seewaves.total_particle_count; i++) {
    	short ptype = g_seewaves.particle_type[i];
    	if(g_seewaves.t[i] == g_seewaves.most_recent_timestamp) {
    		particles_in_current_timestep++;
    	}
        if(ptype == 0) {
    		glColor3f(fluid_color[0], fluid_color[1], fluid_color[2]);
        } else if(ptype == 16) {
    		glColor3f(boundary_color[0], boundary_color[1], boundary_color[2]);
        } else if(ptype == 32) {
    		glColor3f(piston_color[0], piston_color[1], piston_color[2]);
        } else if(ptype == 48) {
    		glColor3f(paddle_color[0], paddle_color[1], paddle_color[2]);
        } else if(ptype == 64) {
    		glColor3f(gate_color[0], gate_color[1], gate_color[2]);
        } else if(ptype == 80) {
    		glColor3f(object_color[0], object_color[1], object_color[2]);
        } else if(ptype == 96) {
    		glColor3f(testpoint_color[0], testpoint_color[1], testpoint_color[2]);
        } else if(ptype == 256) {
    		glColor3f(surface_color[0], surface_color[1], surface_color[2]);
    	} else {
        	glColor3f(0.5, 0.5, 0.5);
    	}
		glVertex3f(g_seewaves.x[i], g_seewaves.z[i], g_seewaves.y[i]);
    }
    glEnd();

    /* render world box (render last for opacity to work */
    glColor4f(0.0, 0.0, 0.0, 0.5);
    glPushMatrix();
    render_box(g_seewaves.world_origin, g_seewaves.world_size);
    glPopMatrix();

    /* is heads up display enabled? */
    if (g_seewaves.view_options & (1 << HEADS_UP)) {
        /* status message buffer */
        char status_msg[1024];
        double loss = 0.0;
        GLfloat y_inc = 20.0f;
        GLfloat y = 10.0f;
        GLfloat x = 10.0f;
        GLfloat rotation_center[3] = { 0.0, 0.0, 0.0 };

    	glPushMatrix();

    	if(g_seewaves.rotation_center[0] != UNDEFINED_PARTICLE) {
        	rotation_center[0] = g_seewaves.rotation_center[0];
        	rotation_center[1] = g_seewaves.rotation_center[1];
        	rotation_center[2] = g_seewaves.rotation_center[2];
        }

        /* set font color */
        glColor3f(FONT_GRAY, FONT_GRAY, FONT_GRAY);

    	/* switch to ortho mode */
    	push_ortho();

    	/* render network status */
    	if(g_seewaves.total_particle_count == 0) {
    		loss = 0.0;
    	} else {
    		loss = particles_in_current_timestep /
    				g_seewaves.total_particle_count * 100.0;
    	}
    	sprintf(status_msg, "network: outgoing(%s:%i:%i) incoming(%s:%i:%i)",
    			g_seewaves.gpusph_host, g_seewaves.gpusph_port,
    			g_seewaves.heartbeats_sent,
    			g_seewaves.data_host, g_seewaves.data_port,
    			g_seewaves.packets_received);
    	render_string(x, y, 0.5f, status_msg);
    	y += y_inc;

    	/* render model status */
    	sprintf(status_msg, "model: particles(%i, %i, %.2f%%) time(%.3fs) steps(%i) id(%u)",
    			g_seewaves.total_particle_count,
    			particles_in_current_timestep, loss,
    			g_seewaves.most_recent_timestamp,
    			g_seewaves.total_timesteps,
    			g_seewaves.model_id);
    	render_string(x, y, 0.5f, status_msg);
    	y += y_inc;

    	/* render camera status (rotated to match model) */
    	sprintf(status_msg,
    			"camera: eye(%.2f, %.2f, %.2f) eye_ctr(%.2f, %.2f, %.2f) rot_ctr(%.2f, %.2f, %.2f) rot(%.2f, %.2f)",
    			eye[0], eye[2], eye[1],
    			target[0], target[2], target[1],
    			rotation_center[0], rotation_center[2], rotation_center[1],
    			g_seewaves.model_rotation[0], g_seewaves.model_rotation[1]);
    	render_string(x, y, 0.5f, status_msg);

    	/* switch back */
    	pop_ortho();
    	glPopMatrix();
    }

#ifdef LOCK
    /* unlock data */
    if ((err = pthread_mutex_unlock(&g_seewaves.lock))) {
        fprintf(stderr, "Error unlocking mutex: %i\n", err);
    }
#endif

    /* handle fading text */
    if(g_seewaves.fade_start != 0) {
    	double diff;
    	double fade_duration = 0.5;
    	time_t now;
    	time(&now);
    	/* gettimeofday to get ms resolution */
    	diff = difftime(now, g_seewaves.fade_start);
    	if(diff >= g_seewaves.fade_duration) {
    		g_seewaves.fade_start = 0;
    		free(g_seewaves.fade_text);
    		g_seewaves.fade_duration = 0.0;
    	} else if((g_seewaves.fade_duration - diff) <= fade_duration) {
    		/* fade */
    		printf("will fade here\n");fflush(stdout);
    	} else {
            /* set font color */
            glColor3f(FONT_GRAY, FONT_GRAY, FONT_GRAY);

        	/* switch to ortho mode */
        	push_ortho();

        	render_string(g_seewaves.fade_position[0], g_seewaves.fade_position[1],
        			g_seewaves.fade_position[2], g_seewaves.fade_text);

        	pop_ortho();
    	}
    }

    /* check for OpenGL errors*/
    while((err = glGetError()) != GL_NO_ERROR) {
    	fprintf(stderr, "OpenGL error: %s\n", gluErrorString(err));
    }
    glPopMatrix();
    glFlush();
    return(1);
}

void opengl_pos_from_mouse_pos(int mx, int my, GLdouble *x, GLdouble *y,
GLdouble *z) {
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

void GLFWCALL on_mouse_button(int button, int action) {
	g_seewaves.mouse_button = button;
	g_seewaves.mouse_button_action = action;
	if((g_seewaves.mouse_button == GLFW_MOUSE_BUTTON_LEFT) &&
		(g_seewaves.mouse_button_action == GLFW_PRESS)) {
		g_seewaves.arcball_last_rotation = g_seewaves.arcball_this_rotation;
		arcball_click(&g_seewaves.arcball, g_seewaves.mouse_x, g_seewaves.mouse_y);
	}
}

/*
 * Called when mouse is moved
 */
void GLFWCALL on_mouse(int x, int y) {
	g_seewaves.mouse_x = x;
	g_seewaves.mouse_y = y;
	if((g_seewaves.mouse_button == GLFW_MOUSE_BUTTON_LEFT) &&
		(g_seewaves.mouse_button_action == GLFW_PRESS)) {
		arcball_drag(&g_seewaves.arcball, x, y, &g_seewaves.arcball_rotation);
		Matrix m = Quaternion_toMatrix(g_seewaves.arcball_rotation);
		Matrix_withMatrix(&g_seewaves.arcball_this_rotation, &m);
		Matrix_multiply(&g_seewaves.arcball_this_rotation, g_seewaves.arcball_last_rotation);
		Matrix_withMatrix(&g_seewaves.arcball_transform, &g_seewaves.arcball_this_rotation);
	}
}

void GLFWCALL on_mouse_wheel(int pos) {
	/* calculate difference from previous mouse wheel setting */
#ifdef __APPLE__
	int diff = g_seewaves.mouse_wheel_pos - pos;
#else
	int diff = pos - g_seewaves.mouse_wheel_pos;
#endif
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

/*
Callback when a character key is pressed or released.
Note, on OSX, action is always GLFW_PRESS and function only called on press
whereas glfwSetKeyCallback() behaves as expected.

@param  key Key code
@param	action	Either GLFW_RELEASE or GLFW_PRESS (see note above)
*/
void GLFWCALL on_char(int key, int action) {
	/* eliminate compiler warning since we don't use action */
	(void)action;

	/* which key was pressed? */
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
        	/* look at X side */
        	float eye[3];

        	g_seewaves.model_pan[0] = 0.0;
        	g_seewaves.model_pan[1] = 0.0;
        	g_seewaves.model_rotation[0] = 0.0;
        	g_seewaves.model_rotation[1] = 0.0;
        	eye[0] = -2.0;
        	eye[1] = g_seewaves.world_origin[2] + (g_seewaves.world_size[2] / 2);
        	eye[2] = g_seewaves.world_origin[1] + (g_seewaves.world_size[1] / 2);
        	camera_set_raw(eye[0], eye[1], eye[2],
        			0.0, 1.0, 0.0,
        			g_seewaves.rotation_center[0], g_seewaves.rotation_center[1],
        			g_seewaves.rotation_center[2]);
        	break;
        }
        case 'Y': {
        	/* look at Y side */
        	float eye[3];

        	g_seewaves.model_pan[0] = 0.0;
        	g_seewaves.model_pan[1] = 0.0;
        	g_seewaves.model_rotation[0] = 0.0;
        	g_seewaves.model_rotation[1] = 0.0;
        	eye[0] = g_seewaves.world_origin[0] + (g_seewaves.world_size[0] / 2);
        	eye[1] = g_seewaves.world_origin[2] + (g_seewaves.world_size[2] / 2);
        	eye[2] = 2.0;

        	camera_set_raw(eye[0], eye[1], eye[2],
        			0.0, 1.0, 0.0,
        			g_seewaves.rotation_center[0], g_seewaves.rotation_center[1],
        			g_seewaves.rotation_center[2]);
        	break;
        }
        case 'Z': {
        	/* look along Z axis */
        	camera_set_raw(5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        	break;
        }
        case '0': {
        	/* reset camera */
        	g_seewaves.model_rotation[0] = 0.0;
        	g_seewaves.model_rotation[1] = 0.0;
        	camera_reset();
        	break;
        }
        case 'c': {
        	/* cycle through camera/view modes */
        	render_fading_text(10.0, g_seewaves.viewport_main[3] - 40.0, 0.5,
                "Hello, world!", 2.0);
        	break;
        }
        case 'a': {
        	g_seewaves.view_options ^= 1 << AXES;
        	g_seewaves.view_options ^= 1 << ROTATION_AXES;
        	break;
        }
        case 'g': {
        	g_seewaves.view_options ^= 1 << GRID;
        	break;
        }
        case '?': {
        	g_seewaves.show_help = 1;
        	break;
        }
        default:
            /* unhandled key */
            break;
    }
}

void physics_update(long usec) {
    (void)usec;
}

/*
Called on window resize event.
Recalculates all dependencies.

@param	w	new width
@param	h	new height
*/
void on_resize(int w, int h) {
	/* cache the new window size internally */
	cfg_set_int(&g_seewaves.config, CFG_WIN_WIDTH, w);
	cfg_set_int(&g_seewaves.config, CFG_WIN_HEIGHT, h);

	/* recalculate viewport sizes */
    g_seewaves.viewport_main[0] = 0;
    g_seewaves.viewport_main[1] = 0;
    g_seewaves.viewport_main[2] = w;
    g_seewaves.viewport_main[3] = h - 50.0;

    g_seewaves.viewport_toolbar[0] = 0;
    g_seewaves.viewport_toolbar[1] = h - 50.0;
    g_seewaves.viewport_toolbar[2] = w;
    g_seewaves.viewport_toolbar[3] = 50.0;

    arcball_set_bounds(&g_seewaves.arcball, (float)w, (float)h);
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
    if (!glfwOpenWindow(get_int(CFG_WIN_WIDTH),
    	get_int(CFG_WIN_HEIGHT),
        g_seewaves.red_bits,
        g_seewaves.green_bits,
        g_seewaves.blue_bits,
        g_seewaves.alpha_bits,
        g_seewaves.depth_bits,
        g_seewaves.stencil_bits,
        g_seewaves.display_mode)) {
        return(-1);
    }

    glfwSetWindowTitle(get_string(CFG_WIN_TITLE));

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
    struct timeval t_start, t_end;
    gettimeofday(&t_start, NULL);
    while (g_seewaves.flag_exit_main_loop != 1) {
        /* did user close window? */
        if (!glfwGetWindowParam(GLFW_OPENED)) {
            /* yes, gracefully exit */
            g_seewaves.flag_exit_main_loop = 1;
        }

        /* render display */
        gettimeofday(&t_end, NULL);
        long usec_diff = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        physics_update(usec_diff);
        if(display()) {
            /* swap the display buffer */
            glfwSwapBuffers();
        }

        /* give the CPU a break */
        usleep(20);
    }

    /* close the underlying sockets, results in threads exiting gracefully */
    close(g_seewaves.data_socket_fd);
    close(g_seewaves.heartbeat_socket_fd);
    shutdown(g_seewaves.data_socket_fd, SHUT_RDWR);
    shutdown(g_seewaves.heartbeat_socket_fd, SHUT_RDWR);

    /* wait for threads to finish */
    if ((err = pthread_join(g_seewaves.data_thread, NULL))) {
        PT_ERR_MSG("pthread_join(data_thread)", err);
    }
    if ((err = pthread_join(g_seewaves.heartbeat_thread, NULL))) {
        PT_ERR_MSG("pthread_join(heartbeat_thread)", err);
    }

    /* terminate glfw */
    glfwTerminate();

    if(g_seewaves.verbosity) {
        fprintf(stdout, "Seewaves exiting\n");
        fflush(stdout);
    }

    cfg_close(&g_seewaves.config);
    exit(EXIT_SUCCESS);
}
