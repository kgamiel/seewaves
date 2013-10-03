/*
 * ArcBall.c
 *
 *  Created on: Oct 1, 2013
 *      Author: kgamiel
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "ArcBall.h"

/* locals */
static void arcball_map_to_sphere(arcball_t *a, float x, float y, Vector *new_vector);


static void arcball_map_to_sphere(arcball_t *a, float x, float y, Vector *new_vector) {
	float tx = x;
	float ty = y;

	tx = (tx * a->adjust_width) - 1.0f;
	ty = 1.0f - (ty * a->adjust_height);

	float length = (tx * tx) + (ty * ty);

	if(length > 1.0f) {
		float norm = 1.0f / sqrtf(length);
		new_vector->x = tx * norm;
		new_vector->y = ty * norm;
		new_vector->z = 0.0f;
	} else {
		new_vector->x = tx;
		new_vector->y = ty;
		new_vector->z = sqrtf(1.0f - length);
	}
}

void arcball_init(arcball_t *a, float width, float height) {
	memset(a, 0, sizeof(arcball_t));
	arcball_set_bounds(a, width, height);
}

void arcball_set_bounds(arcball_t *a, float width, float height) {
	a->adjust_width = 1.0f / ((width  - 1.0f) * 0.5f);
	a->adjust_height = 1.0f / ((height - 1.0f) * 0.5f);
}

void arcball_click(arcball_t *a, float x, float y) {
	arcball_map_to_sphere(a, x, y, &a->saved_click_vector);
}

void arcball_drag(arcball_t *a, float x, float y, Quaternion *new_rotation) {
	arcball_map_to_sphere(a, x, y, &a->saved_drag_vector);
	if(!new_rotation) {
		return;
	}
	Vector perpendicular = Vector_cross(a->saved_click_vector, a->saved_drag_vector);
	if(Vector_magnitude(perpendicular) > Epsilon) {
		new_rotation->x = perpendicular.x;
		new_rotation->y = perpendicular.y;
		new_rotation->z = perpendicular.z;
		new_rotation->w = Vector_dot(a->saved_click_vector, a->saved_drag_vector);
	} else {
		new_rotation->x = 0.0f;
		new_rotation->y = 0.0f;
		new_rotation->z = 0.0f;
		new_rotation->w = 0.0f;
	}
}
