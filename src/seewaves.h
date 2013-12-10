#ifndef SEEWAVES_H
#define SEEWAVES_H

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <unistd.h>
#include <GL/glut.h>
#endif
#include "ArcBall.h"
#include "cfg.h"
#include "Matrix.h"

/* Versioning */
#define VERSION_HIGH 0
#define VERSION_LOW 13

#define UNDEFINED_PARTICLE -1.0

/*
glfw mouse wheel behavior currently different in OSX v. Linux, scheduled for
fix in version 3.0.
*/
#ifdef __APPLE__
#define CAMERA_TRANSLATE_SCALER 0.01
#else
#define CAMERA_TRANSLATE_SCALER 1.0
#endif
#define FONT_GRAY 0.5

typedef enum { HEADS_UP, AXES, GRID, ROTATION_AXES } seewaves_view_option_t;
typedef enum { SHIFT } seewaves_key_option_t;

/* define all application configuration settings here */
#define CFG_WIN_TITLE	"window.title"
#define CFG_WIN_X		"window.x"
#define CFG_WIN_Y		"window.y"
#define CFG_WIN_WIDTH	"window.width"
#define CFG_WIN_HEIGHT	"window.height"
#define CFG_EYE_POS		"eye.position"
#define CFG_EYE_UP		"eye.up"
#define CFG_EYE_TARGET	"eye.target"
#define CFG_ZNEAR		"znear"
#define CFG_ZFAR		"zfar"

/* Global application data structure */
typedef struct {
	/* configuration */
	cfg_t config;
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
    double *x;
    /* array of y position of all particles, total_particle_cnt long */
    double *y;
    /* array of z position of all particles, total_particle_cnt long */
    double *z;
    /* array of w (mass) of all particles, total_particle_cnt long */
    double *w;
    /* array of particle_type of all particles, total_particle_cnt long */
    short *particle_type;
    /* array of t (timestamp) of all particles, total_particle_cnt long */
    float *t;
    /* particle flag */
    unsigned int *flag;
    /* total number of packets received from server */
    int packets_received;
    /* main application loop exit flag */
    int flag_exit_main_loop;
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
    /* window dimensions */
    GLint window[4];
    /* most recent timestamp */
    float most_recent_timestamp;
    /* total timesteps */
    int total_timesteps;
    /* UDP receiver buffer size (optionally set by user) */
    int udp_buffer_size;
    /* view options bit string */
    unsigned char view_options;
    /* mouse x position */
    float mouse_x;
    /* mouse y position */
    float mouse_y;
    /* mouse button id */
    int mouse_button;
    /* mouse button action */
    int mouse_button_action;
    /* mouse wheel position */
    int mouse_wheel_pos;
    /* key options */
    unsigned char key_options;
    /* text to fade */
    char *fade_text;
    /* text to fade start time */
    time_t fade_start;
    /* how long should the fade last */
    double fade_duration;
    /* position of fading text */
    GLfloat fade_position[3];
    /* contains range of glLineWidth() values */
	GLfloat line_width_range[2];
	/* contains step size for glLineWidth() values */
	GLfloat line_width_step;
    /* contains range of glPointSize() values */
	GLfloat point_size_range[2];
	/* contains step size for glPointSize() values */
	GLfloat point_size_step;
	/* toolbar viewport dimensions */
	GLfloat viewport_toolbar[4];
	/* main viewport dimensions */
	GLfloat viewport_main[4];
	/* center of rotation */
	float rotation_center[3];
	/* origin of world */
	float world_origin[3];
	/* size of world */
	float world_size[3];
	/* mouse pointer location when clicked */
	int mouse_pressed_at[2];
	/* model rotation */
	float model_rotation[3];
	/* model pan */
	float model_pan[3];
	/* rotation view */
	arcball_t arcball;
	Quaternion arcball_rotation;
	Matrix arcball_transform;
	Matrix arcball_last_rotation;
	Matrix arcball_this_rotation;
} seewaves_t;

/* formatting flag */
typedef enum { BASIC, FULL } seewaves_format_t;


#endif
