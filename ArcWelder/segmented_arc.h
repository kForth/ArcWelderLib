////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arc Welder: Anti-Stutter Library
//
// Compresses many G0/G1 commands into G2/G3(arc) commands where possible, ensuring the tool paths stay within the specified resolution.
// This reduces file size and the number of gcodes per second.
//
// Uses the 'Gcode Processor Library' for gcode parsing, position processing, logging, and other various functionality.
//
// Copyright(C) 2021 - Brad Hochgesang
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This program is free software : you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU Affero General Public License for more details.
//
//
// You can contact the author at the following email address: 
// FormerLurker@pm.me
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include "segmented_shape.h"
#define GCODE_CHAR_BUFFER_SIZE 1000

class segmented_arc :
	public segmented_shape
{
public:
	segmented_arc();
	segmented_arc(
		bool allow_3d_shapes = DEFAULT_ALLOW_3D_ARCS,
		int min_segments = DEFAULT_MIN_SEGMENTS,
		int max_segments = DEFAULT_MAX_SEGMENTS,
		double mm_per_segment = DEFAULT_MM_PER_SEGMENT,
		double resolution_mm = DEFAULT_RESOLUTION_MM,
		double path_tolerance_percent = LENGTH_PERCENT_TOLERANCE_DEFAULT,
		int max_gcode_length = DEFAULT_MAX_GCODE_LENGTH,
		unsigned char default_xyz_precision = DEFAULT_XYZ_PRECISION,
		unsigned char default_e_precision = DEFAULT_E_PRECISION,

		double max_radius_mm = DEFAULT_MAX_RADIUS_MM
	);
	virtual ~segmented_arc();
	virtual double get_length();
	virtual std::string get_gcode() const;
	virtual int get_gcode_length();
	const std::string shape_name = "Arc";

	double get_max_radius() const;
protected:
	virtual bool try_add_point_internal_(printer_point p);
private:
	arc current_arc_;
	double max_radius_mm_;
};															

