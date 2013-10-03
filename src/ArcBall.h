/*
 * ArcBall.h
 *
 *  Created on: Oct 1, 2013
 *      Author: kgamiel
 */

#ifndef ARCBALL_H_
#define ARCBALL_H_

#include "Vector.h"
#include "Quaternion.h"

# define Epsilon 1.0e-5

typedef struct {
	float	adjust_width;
	float	adjust_height;
	Vector	saved_click_vector;
	Vector	saved_drag_vector;
} arcball_t;

void arcball_init(arcball_t *a, float width, float height);
void arcball_set_bounds(arcball_t *a, float width, float height);
void arcball_click(arcball_t *a, float x, float y);
void arcball_drag(arcball_t *a, float x, float y, Quaternion *new_rotation);

#endif /* ARCBALL_H_ */
