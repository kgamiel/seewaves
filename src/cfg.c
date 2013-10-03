/*
 * cfg.c
 *
 *  Created on: Jun 2, 2013
 *      Author: kgamiel
 */

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <ctype.h>
#include "cfg.h"

static char *trim(char *str);


/*
No time for fancy configuration handling, we're replace this with some 3rd party library later.
*/

static char *trim(char *str) {
	char *end;
	while(isspace(*str)) {
		str++;
	}
	if(*str == 0) {
		return(str);
	}
	end = str + strlen(str) - 1;
	while(end > str && isspace(*end)) {
		end--;
	}
	*(end+1) = 0;
	return(str);
}

/*
Open a configuration file.

@param	cfg	pointer to cfg_t structure
@param	filename	filename

@returns CFG_OK on success, otherwise CFG_ERROR
*/
cfg_error_t cfg_open(cfg_t *cfg, const char *filename, cfg_option_t *options) {
	/* file status */
	struct stat status;

	/* clear it out */
	memset(cfg, 0, sizeof(cfg));

	/* keep the filename */
	snprintf(cfg->filename, FILENAME_MAX, filename, "%s");

	/* just check existence */
	if(stat(cfg->filename, &status)) {
		perror(cfg->filename);
		return (CFG_ERROR);
	}

	/* keep the options */
	cfg->options = options;

	return(CFG_OK);
}

/*
Get integer value.

@param	cfg	pointer to cfg_t structure
@param	name	parameter name
@param	value	value pointer

@returns int value
*/
int cfg_get_int(cfg_t *cfg, const char *name, int default_value) {
	char buf[64];
	cfg_get_string(cfg, name, buf, sizeof(buf), "xxx");
	if(!strcmp(buf, "xxx")) {
		return(default_value);
	}
	return(atoi(buf));
}

float cfg_get_float(cfg_t *cfg, const char *name, float default_value) {
	char buf[64];
	cfg_get_string(cfg, name, buf, sizeof(buf), "xxx");
	if(!strcmp(buf, "xxx")) {
		return(default_value);
	}
	return(atof(buf));
}

char *cfg_get_string(cfg_t *cfg, const char *name, char *value, int max_len,
		const char *default_value) {
	FILE *fp;
	int line;

	snprintf(value, max_len, "%s", default_value);

	assert((cfg != NULL) && (cfg->filename != NULL) && (cfg->filename[0] != '\0'));
	if((fp = fopen(cfg->filename, "r")) == NULL) {
		perror(cfg->filename);
		return(value);
	}
	/* read each line of file until target is found or end is reached */
	for( line = 1; ; line++) {
		/* buffer for line */
		char line_buf[CFG_MAX_LINE];
		char lname[CFG_MAX_LINE];

		/* read line from file */
		if(fgets(line_buf, sizeof(line_buf), fp) == NULL) {
			break;
		}

		/* trim pre and post white space */
		trim(line_buf);

		/* skip empty lines and comments */
		if(line_buf[0] == '\0' || line_buf[0] == '#') {
			continue;
		}
		/* get the configuration option name */
		if(sscanf(line_buf, "%s", lname) == 1) {
			/* is it the one we want? */
			if(!strcmp(name, lname)) {
				/* yes, copy and return */
				int start = strlen(name) + 1;
				snprintf(value, max_len, "%s", &(line_buf[start]));
				trim(value);
				break;
			}
		}
	}
	fclose(fp);
	return(value);
}

cfg_option_t *cfg_get(cfg_t *cfg, const char *name) {
	cfg_option_t *option;
	int i;
	assert(name != NULL && *name != '\0');
	for( i = 0; ; i++) {
		option = &(cfg->options[i]);
		if(option->name == NULL) {
			break;
		}
		if(!strcmp(name, option->name)) {
			return(option);
		}
	}
	return NULL;
}

void cfg_set_int(cfg_t *cfg, char *name, int value) {
	cfg_option_t *option = cfg_get(cfg, name);
	if(option && option->which == INTEGER) {
		option->u.ival = value;
	}
}

cfg_error_t cfg_close(cfg_t *cfg) {
	(void)cfg;
	return(CFG_OK);
}
