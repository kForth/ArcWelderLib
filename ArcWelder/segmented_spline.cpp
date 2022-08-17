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

#include "segmented_spline.h"
#include "utilities.h"
#include "segmented_shape.h"
#include <iostream>
#include <stdio.h>
#include <cmath>

segmented_spline::segmented_spline() : segmented_shape(
  DEFAULT_MIN_SEGMENTS,
  DEFAULT_MAX_SEGMENTS,
  DEFAULT_MM_PER_SEGMENT,
  DEFAULT_RESOLUTION_MM,
  LENGTH_PERCENT_TOLERANCE_DEFAULT,
  DEFAULT_MAX_GCODE_LENGTH
)
{
  allow_3d_shapes_ = DEFAULT_ALLOW_3D_SPLINES;
}

segmented_spline::segmented_spline(
  bool allow_3d_shapes,
  int min_segments,
  int max_segments,
  double mm_per_segment,
  double resolution_mm,
  double path_tolerance_percent,
  int max_gcode_length,
  unsigned char default_xyz_precision,
  unsigned char default_e_precision
) : segmented_shape(
  allow_3d_shapes,
  min_segments,
  max_segments,
  mm_per_segment,
  resolution_mm,
  path_tolerance_percent,
  max_gcode_length,
  default_xyz_precision,
  default_e_precision
)
{

}

segmented_spline::~segmented_spline()
{
}

double segmented_spline::get_length()
{
  return current_spline_.length;
}

bool segmented_spline::try_add_point_internal_(printer_point p)
{
  // If we don't have enough points (at least min_segments) return false
  if (points_.count() < get_min_segments() - 1)
    return false;

  // the circle is new..  we have to test it now, which is expensive :(
  points_.push_back(p);
  double previous_shape_length = original_shape_length_;
  original_shape_length_ += p.distance;
  spline original_spline = current_spline_;
  if (spline::try_create_spline(&points_, &current_spline_, original_shape_length_, resolution_mm_, path_tolerance_percent_, min_segments_, mm_per_segment_, get_xyz_tolerance()))
  {
    bool abort_spline = false;
    if (max_gcode_length_ > 0 && get_gcode_length() > max_gcode_length_)
    {
      abort_spline = true;
      num_gcode_length_exceptions_++;
    }
    if (!abort_spline)
    {
      // TODO: Verify spline arguments
      //if (utilities::is_zero(current_spline_.get_i(), get_xyz_tolerance()) && utilities::is_zero(current_spline_.get_j(), get_xyz_tolerance()))
      //{
      //  // I and J are both 0, which is invalid!  Abort!
      //  abort_spline = true;
      //}
      //else
      if (current_spline_.length < get_xyz_tolerance())
      {
        // the spline length is below our tolerance, abort!
        abort_spline = true;
      }
    }

    if (abort_spline)
    {
      // This spline has been cancelled either due to firmware correction,
      // or because both I and J == 0
      current_spline_ = original_spline;
    }
    else
    {
      if (!is_shape())
      {
        set_is_shape(true);
      }
      return true;
    }
  }
  // Can't create the spline.  Remove the point and remove the previous segment length.
  points_.pop_back();
  original_shape_length_ = previous_shape_length;
  return false;
}

std::string segmented_spline::get_gcode() const
{
  double e = current_spline_.end_point.is_extruder_relative ? e_relative_ : current_spline_.end_point.e_offset;
  double f = current_spline_.start_point.f == current_spline_.end_point.f ? 0 : current_spline_.end_point.f;
  bool has_e = e_relative_ != 0;
  bool has_f = utilities::greater_than_or_equal(f, 1);
  bool has_z = !utilities::is_equal(
    current_spline_.start_point.z, current_spline_.end_point.z, get_xyz_tolerance()
  );

  // TODO: Break large spline into multiple G5 commands

  std::string gcode;

  gcode.reserve(96);

  gcode += "G5";
  // TODO: Limit Gcode Precision based on max_gcode_length


  // Add X, Y, I, J, P, Q
  gcode += " X";
  gcode += utilities::dtos(current_spline_.end_point.x, get_xyz_precision());

  gcode += " Y";
  gcode += utilities::dtos(current_spline_.end_point.y, get_xyz_precision());

  if (has_z)
  {
    gcode += " Z";
    gcode += utilities::dtos(current_spline_.end_point.z, get_xyz_precision());
  }

  // Output I, J, P, and Q, but do NOT check for 0.
  //double i = current_spline_.get_i();
  //gcode += " I";
  //gcode += utilities::dtos(i, get_xyz_precision());

  //double j = current_spline_.get_j();
  //gcode += " J";
  //gcode += utilities::dtos(j, get_xyz_precision());

  //double p = current_spline_.get_p();
  //gcode += " P";
  //gcode += utilities::dtos(p, get_xyz_precision());

  //double q = current_spline_.get_q();
  //gcode += " Q";
  //gcode += utilities::dtos(q, get_xyz_precision());

  // Add E if it appears
  if (has_e)
  {
    gcode += " E";
    gcode += utilities::dtos(e, get_e_precision());
  }

  // Add F if it appears
  if (has_f)
  {
    gcode += " F";
    gcode += utilities::dtos(f, 0);
  }

  //TODO: Handle S term?

  return gcode;

}

int segmented_spline::get_gcode_length()
{
  double e = current_spline_.end_point.is_extruder_relative ? e_relative_ : current_spline_.end_point.e_offset;
  double f = current_spline_.start_point.f == current_spline_.end_point.f ? 0 : current_spline_.end_point.f;
  bool has_e = e_relative_ != 0;
  bool has_f = utilities::greater_than_or_equal(f, 1);
  bool has_z = !utilities::is_equal(
    current_spline_.start_point.z, current_spline_.end_point.z, get_xyz_tolerance()
  );

  int xyz_precision = get_xyz_precision();
  int e_precision = get_e_precision();

  double i = 0.0; // current_spline_.get_i();
  double j = 0.0; // current_spline_.get_j();
  double p = 0.0; // current_spline_.get_p();
  double q = 0.0; // current_spline_.get_q();

  int num_spaces = 4 + (has_z ? 1 : 0) + (has_e ? 1 : 0) + (has_f ? 1 : 0);
  int num_decimal_points = 4 + (has_z ? 1 : 0) + (has_e ? 1 : 0);  // note f has no decimal point
  int num_decimals = xyz_precision * (4 + (has_z ? 1 : 0)) + e_precision * (has_e ? 1 : 0); // Note f is an int
  int num_digits = (
    utilities::get_num_digits(current_spline_.end_point.x, xyz_precision) +
    utilities::get_num_digits(current_spline_.end_point.y, xyz_precision) +
    (has_z ? utilities::get_num_digits(current_spline_.end_point.z, xyz_precision) : 0) +
    (has_e ? utilities::get_num_digits(e, e_precision) : 0) +
    utilities::get_num_digits(i, xyz_precision) +
    utilities::get_num_digits(j, xyz_precision) +
    utilities::get_num_digits(p, xyz_precision) +
    utilities::get_num_digits(q, xyz_precision) +
    (has_f ? utilities::get_num_digits(f, 0) : 0)
    );
  int num_minus_signs = (
    (current_spline_.end_point.x < 0 ? 1 : 0) +
    (current_spline_.end_point.y < 0 ? 1 : 0) +
    (i < 0 ? 1 : 0) +
    (j < 0 ? 1 : 0) +
    (p < 0 ? 1 : 0) +
    (q < 0 ? 1 : 0) +
    (has_e && e < 0 ? 1 : 0) +
    (has_z && current_spline_.end_point.z < 0 ? 1 : 0)
    );

  int num_parameters = 6 + (has_e ? 1 : 0) + (has_z ? 1 : 0) + (has_f ? 1 : 0);
  // Return the length of the gcode.
  int gcode_length = 2 + num_spaces + num_decimal_points + num_digits + num_minus_signs + num_decimals + num_parameters;

  // Keep this around in case we have any future issues with the gcode length calculation
#ifdef Debug
  std::string gcode = get_shape_gcode();
  if (gcode.length() != gcode_length)
  {
    return 9999999;
  }
#endif
  return gcode_length;

}