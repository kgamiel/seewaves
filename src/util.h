/*
 * util.h
 *
 *  Created on: Dec 10, 2013
 *      Author: kgamiel
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "seewaves.h"

void util_get_current_time_string(char *buf, ssize_t max_len);
void util_print_seewaves(seewaves_t *s, seewaves_format_t format, int fd);
int util_get_udp_buffer_size(int sd);

#endif /* UTIL_H_ */
