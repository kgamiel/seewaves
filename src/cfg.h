/*
 * cfg.h
 *
 *  Created on: Jun 2, 2013
 *      Author: kgamiel
 */

#ifndef CFG_H_
#define CFG_H_

/*
No time for fancy configuration handling, we're replace this with some 3rd party library later.
*/

/*
Supported data types
*/
typedef enum { STRING, INTEGER, FLOAT, FLOAT3 } cfg_option_which_t;

/*
Maximum value length as represented in the configuration file
*/
#define MAX_CFG_STRING 512

/*
Each option is represented as a structure of data.
*/
typedef struct {
	/* option name */
	char *name;

	/* option description */
	char *description;

	/* option type */
	cfg_option_which_t which;

	/* option value, depending on type */
	union {
		char sval[MAX_CFG_STRING];
		int ival;
		float fval;
		float f3val[3];
	} u;

	/* option default value, depending on type */
	union {
		char *sval;
		int ival;
		float fval;
		float f3val[3];
	} d;
} cfg_option_t;

/*
Config structure
*/
typedef struct {
	char filename[FILENAME_MAX];
	cfg_option_t *options;
} cfg_t;

#define CFG_MAX_LINE 1024

typedef enum { CFG_OK, CFG_ERROR } cfg_error_t;

cfg_error_t cfg_open(cfg_t *cfg, const char *filename, cfg_option_t *options);
int cfg_get_int(cfg_t *cfg, const char *name, int default_value);
float cfg_get_float(cfg_t *cfg, const char *name, float default_value);
char *cfg_get_string(cfg_t *cfg, const char *name, char *value, int max_len, const char *default_value);
cfg_option_t *cfg_get(cfg_t *cfg, const char *name);
void cfg_set_int(cfg_t *cfg, char *name, int value);
cfg_error_t cfg_close(cfg_t *cfg);

#endif /* CFG_H_ */
