// Copyright 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VELODYNE_DRIVER__TIME_CONVERSION_HPP_
#define VELODYNE_DRIVER__TIME_CONVERSION_HPP_

#include <rclcpp/time.hpp>
#include <fstream>

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 *
 * @param stamp timestamp recovered from velodyne
 * @param nominal_stamp time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
inline
rclcpp::Time resolveHourAmbiguity(rclcpp::Node * private_nh_, const rclcpp::Time & stamp, const rclcpp::Time & nominal_stamp)
{
  const int HALFHOUR_TO_SEC = 1800;
  rclcpp::Time retval = stamp;
  int debug_val = 0; 

  if (nominal_stamp.seconds() > stamp.seconds()) {
    if (nominal_stamp.seconds() - stamp.seconds() > HALFHOUR_TO_SEC) {
      retval = rclcpp::Time(retval.seconds() + 2 * HALFHOUR_TO_SEC);

      debug_val = 1; 
    }
  } else if (stamp.seconds() - nominal_stamp.seconds() > HALFHOUR_TO_SEC) {
    retval = rclcpp::Time(retval.seconds() - 2 * HALFHOUR_TO_SEC);

    debug_val = -1; 
  }

  // Log debug info
  RCLCPP_INFO(private_nh_->get_logger(),
    "resolveHourAmbiguity → debug_val: %d",
    debug_val);

  return retval;
}
// This function *focus 
inline
rclcpp::Time rosTimeFromGpsTimestamp(rclcpp::Node * private_nh_, rclcpp::Time & time_nom, const uint8_t * const data)
{
  
  // time_nom is used to recover the hour
  const int HOUR_TO_SEC = 3600;
  // time for each packet is a 4 byte uint
  // It is the number of microseconds from the top of the hour
  uint32_t usecs =
    static_cast<uint32_t>(
    ((uint32_t) data[3]) << 24 |
      ((uint32_t) data[2]) << 16 |
      ((uint32_t) data[1]) << 8 |
      ((uint32_t) data[0]));

  // Error is most likely here where it calcs current hour.
  uint32_t cur_hour_seg1 =  time_nom.nanoseconds() / 1000000000;
  uint32_t cur_hour = cur_hour_seg1 / HOUR_TO_SEC;
  
  double stamp_seg1 = cur_hour * HOUR_TO_SEC;
  double stamp_seg2 = usecs / 1000000;
  double stamp_seg3 = (usecs % 1000000) * 1000;

  auto stamp = rclcpp::Time(
    (stamp_seg1) + (stamp_seg2),
    stamp_seg3);

  rclcpp::Time retval = resolveHourAmbiguity(private_nh_ ,stamp, time_nom);

  RCLCPP_INFO(private_nh_->get_logger(),
    "rosTimeFromGpsTimestamp → usecs: %u, cur_hour_seg1: %u, cur_hour: %u, stamp_seg1: %.0f, stamp_seg2: %.6f, stamp_seg3: %.0f, stamp: %.6f, retval: %.6f",
    static_cast<unsigned int>(usecs),
    static_cast<unsigned int>(cur_hour_seg1),
    static_cast<unsigned int>(cur_hour),
    stamp_seg1,
    stamp_seg2,
    stamp_seg3,
    stamp.seconds(),
    retval.seconds());

  RCLCPP_INFO(private_nh_->get_logger(), "");
    
  return retval;
}

#endif  // VELODYNE_DRIVER__TIME_CONVERSION_HPP_
