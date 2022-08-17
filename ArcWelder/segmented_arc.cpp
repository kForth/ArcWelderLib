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

#include "segmented_arc.h"
#include "utilities.h"
#include "segmented_shape.h"
#include <iostream>
#include <stdio.h>
#include <cmath>

segmented_arc::segmented_arc() : segmented_shape(
  DEFAULT_MIN_SEGMENTS,
  DEFAULT_MAX_SEGMENTS,
  DEFAULT_MM_PER_SEGMENT,
  DEFAULT_RESOLUTION_MM,
  LENGTH_PERCENT_TOLERANCE_DEFAULT,
  DEFAULT_MAX_GCODE_LENGTH
)
{
  max_radius_mm_ = DEFAULT_MAX_RADIUS_MM;
}

segmented_arc::segmented_arc(
  bool allow_3d_shapes,
  int min_segments,
  int max_segments,
  double mm_per_segment,
  double resolution_mm,
  double path_tolerance_percent,
  int max_gcode_length,
  unsigned char default_xyz_precision,
  unsigned char default_e_precision,
  double max_radius_mm
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
  max_radius_mm_ = max_radius_mm;
  if (max_radius_mm > DEFAULT_MAX_RADIUS_MM) {
    max_radius_mm_ = DEFAULT_MAX_RADIUS_MM;
  }
  
}

segmented_arc::~segmented_arc()
{
}

double segmented_arc::get_max_radius() const
{
  return max_radius_mm_;
}

double segmented_arc::get_length()
{
  return current_arc_.length;
}

bool segmented_arc::try_add_point_internal_(printer_point p)
{
  // If we don't have enough points (at least min_segments) return false
  if (points_.count() < get_min_segments() - 1)
    return false;

  // the circle is new..  we have to test it now, which is expensive :(
  points_.push_back(p);
  double previous_shape_length = original_shape_length_;
  original_shape_length_ += p.distance;
  arc original_arc = current_arc_;
  if (arc::try_create_arc(
    points_,
    current_arc_,
    original_shape_length_,
    max_radius_mm_,
    resolution_mm_,
    path_tolerance_percent_,
    get_xyz_tolerance(),
    allow_3d_shapes_)
  )
  {
    bool abort_arc = false;
    if (max_gcode_length_ > 0 && get_gcode_length() > max_gcode_length_)
    {
      abort_arc = true;
      num_gcode_length_exceptions_++;
    }
    if (min_segments_ > 0 && mm_per_segment_ > 0)
    {
      // Apply firmware compensation
      // See how many arcs will be interpolated
      double circumference = 2.0 * PI_DOUBLE * current_arc_.radius;
      // TODO: Should this be ceil?
      int num_segments = (int)utilities::floor(circumference / min_segments_);
      if (num_segments < min_segments_) {
        //num_segments = (int)std::ceil(circumference/approximate_length) * (int)std::ceil(approximate_length / mm_per_arc_segment);
        // TODO: Should this be ceil?
        num_segments = (int)utilities::floor(circumference / original_shape_length_);
        if (num_segments < min_segments_) {
          abort_arc = true;
          num_firmware_compensations_++; 
        }
      }
    }
    if (!abort_arc)
    {
      if (utilities::is_zero(current_arc_.get_i(), get_xyz_tolerance()) && utilities::is_zero(current_arc_.get_j(), get_xyz_tolerance()))
      {
        // I and J are both 0, which is invalid!  Abort!
        abort_arc = true;
      }
      else if (current_arc_.length < get_xyz_tolerance())
      {
        // the arc length is below our tolerance, abort!
        abort_arc = true;
      }
    }
    
    if (abort_arc)
    {
      // This arc has been cancelled either due to firmware correction,
      // or because both I and J == 0
      current_arc_ = original_arc;
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
  // Can't create the arc.  Remove the point and remove the previous segment length.
  points_.pop_back();
  original_shape_length_ = previous_shape_length;
  return false;
}

std::string segmented_arc::get_gcode() const
{
  std::string gcode;
  double e = current_arc_.end_point.is_extruder_relative ? e_relative_ : current_arc_.end_point.e_offset;
  double f = current_arc_.start_point.f == current_arc_.end_point.f ? 0 : current_arc_.end_point.f;
  bool has_e = e_relative_ != 0;
  bool has_f = utilities::greater_than_or_equal(f, 1);
  bool has_z = allow_3d_shapes_ && !utilities::is_equal(
    current_arc_.start_point.z, current_arc_.end_point.z, get_xyz_tolerance()
  );
  gcode.reserve(96);

  if (current_arc_.angle_radians < 0)
  {
    gcode += "G2";
  }
  else
  {
    gcode += "G3";

  }
  // TODO: Limit Gcode Precision based on max_gcode_length


  // Add X, Y, I and J
  gcode += " X";
  gcode += utilities::dtos(current_arc_.end_point.x, get_xyz_precision());
  
  gcode += " Y";
  gcode += utilities::dtos(current_arc_.end_point.y, get_xyz_precision());
  
  if (has_z)
  {
    gcode += " Z";
    gcode += utilities::dtos(current_arc_.end_point.z, get_xyz_precision());
  }

  // Output I and J, but do NOT check for 0.  
  // Simplify 3d has issues visualizing G2/G3 with 0 for I or J
  // and until it is fixed, it is not worth the hassle.
  double i = current_arc_.get_i();
  gcode += " I";
  gcode += utilities::dtos(i, get_xyz_precision());

  double j = current_arc_.get_j();
  gcode += " J";
  gcode += utilities::dtos(j, get_xyz_precision());

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

  return gcode;

}

int segmented_arc::get_gcode_length()
{
  double e = current_arc_.end_point.is_extruder_relative ? e_relative_ : current_arc_.end_point.e_offset;
  double f = current_arc_.start_point.f == current_arc_.end_point.f ? 0 : current_arc_.end_point.f;
  bool has_e = e_relative_ != 0;
  bool has_f = utilities::greater_than_or_equal(f, 1);
  bool has_z = allow_3d_shapes_ && !utilities::is_equal(
    current_arc_.start_point.z, current_arc_.end_point.z, get_xyz_tolerance()
  );
  
  int xyz_precision = get_xyz_precision();
  int e_precision = get_e_precision();

  double i = current_arc_.get_i();
  double j = current_arc_.get_j();

  int num_spaces = 4 + (has_z ? 1 : 0) + (has_e ? 1 : 0) + (has_f ? 1 : 0);
  int num_decimal_points = 4 + (has_z ? 1 : 0) + (has_e ? 1 : 0);  // note f has no decimal point
  int num_decimals = xyz_precision * (4 + (has_z ? 1 : 0)) + e_precision * (has_e ? 1 : 0); // Note f is an int
  int num_digits = (
    utilities::get_num_digits(current_arc_.end_point.x, xyz_precision) +
    utilities::get_num_digits(current_arc_.end_point.y, xyz_precision) +
    (has_z ? utilities::get_num_digits(current_arc_.end_point.z, xyz_precision) : 0) +
    (has_e ? utilities::get_num_digits(e, e_precision) : 0) +
    utilities::get_num_digits(i, xyz_precision) +
    utilities::get_num_digits(j, xyz_precision) +
    (has_f ? utilities::get_num_digits(f,0) : 0)
  );
  int num_minus_signs = (
    (current_arc_.end_point.x < 0 ? 1 : 0) + 
    (current_arc_.end_point.y < 0 ? 1 : 0) +
    (i < 0 ? 1 : 0) +
    (j < 0 ? 1 : 0) +
    (has_e && e < 0 ? 1 : 0) +
    (has_z && current_arc_.end_point.z < 0 ? 1 : 0)
  );

  int num_parameters = 4 + (has_e ? 1 : 0) + (has_z ? 1: 0) + (has_f ? 1: 0);
  // Return the length of the gcode.
  int gcode_length = 2 + num_spaces + num_decimal_points + num_digits + num_minus_signs + num_decimals + num_parameters;
  
  // Keep this around in case we have any future issues with the gcode length calculation
  #ifdef Debug
  std::string gcode = get_gcode();
  if (gcode.length() != gcode_length)
  {
    return 9999999; 
  }
  #endif
  return gcode_length;

}

/*
* This is an older implementation using ostringstream.  It is substantially slower.
* Keep this around in case there are problems with the custom dtos function
std::string segmented_arc::get_gcode_(bool has_e, double e, double f) const
{
  static std::ostringstream gcode;
  gcode.str("");
  gcode << std::fixed;

  if (current_arc_.angle_radians < 0)
  {
    gcode << "G2";
  }
  else
  {
    gcode << "G3";

  }
  gcode << std::setprecision(get_xyz_precision());
  // Add X, Y, I and J
  if (!utilities::is_zero(current_arc_.end_point.x, get_xyz_precision()))
  {
    gcode << " X" << current_arc_.end_point.x;
  }

  if (!utilities::is_zero(current_arc_.end_point.y, get_xyz_precision()))
  {
    gcode << " Y" << current_arc_.end_point.y;
  }

  if (allow_3d_shapes_)
  {
    // We may need to add a z coordinate
    double z_initial = current_arc_.start_point.z;
    double z_final = current_arc_.end_point.z;
    if (!utilities::is_equal(z_initial, z_final, get_xyz_tolerance()))
    {
      // The z axis has changed within the precision of the gcode coordinates
      gcode << " Z" << current_arc_.end_point.z;
    }
  }

  // Output I and J, but do NOT check for 0.
  // Simplify 3d has issues visualizing G2/G3 with 0 for I or J
  // and until it is fixed, it is not worth the hassle.
  double i = current_arc_.get_i();
  gcode << " I" << i;

  double j = current_arc_.get_j();
  gcode << " J" << j;

  // Add E if it appears
  if (has_e)
  {
    gcode << std::setprecision(get_e_precision());
    gcode << " E" << e;
  }

  // Add F if it appears
  if (utilities::greater_than_or_equal(f, 1))
  {
    gcode << std::setprecision(0);
    gcode << " F" << f;
  }

  return gcode.str();

}
*/