/*
 * Copyright (C) GROUP 1
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/master_corner_avoider/master_corner_avoider.c"
 * @author GROUP 1
 * Links the corner detector software to the camera and makes it see the corners
 */

#include "modules/master_corner_avoider/master_corner_avoider.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/lib/vision/fast_rosten.h"

struct video_listener *mca_listener = NULL;


// Corner Detector Settings
uint8_t  threshold 				= 12;
uint16_t min_dist 				= 12;
uint16_t x_padding 				= 12;
uint16_t y_padding 				= 12;
uint16_t num_corners 			= 12;

uint16_t ret_corners_length 	= MAX_POINTS;
struct   point_t ret_corners[MAX_POINTS];

// Result
int corner_count = 0;

// Function
struct image_t *mca_function(struct image_t *img);
struct image_t *mca_function(struct image_t *img)
{
	// Corner Detector
	fast9_detect(img, threshold, min_dist, 
								x_padding, y_padding, &num_corners, 
								&ret_corners_length, &ret_corners[0]
								);
	printf("num_corners: %d\n", num_corners);
	return img;
}

// Initiate camera that is used to detect corners
void mca_init(void) 
{
	mca_listener = cv_add_to_device(&CORNERDETECTOR_CAMERA, mca_function);
}