/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "rosbag/time_manager.h"
#include "rosgraph_msgs/Clock.h"

namespace rosbag {

TimeManager::TimeManager(
    ros::NodeHandle const& nh,
    ros::Time const& bag_start_time,
    double const& time_scale)
{
    time_scale_ = time_scale;
    bag_ref_time_ = bag_start_time_;
    ros::WallTime ref = ros::WallTime::now();
    wall_ref_time_ = ros::Time(ref.sec, ref.nsec);
    paused_ = true;
    time_pub_ = nh.advertise<rosgraph_msgs::Clock>("clock",1);
}

void TimeManager::setPublishFrequency(double publish_frequency)
{
    publish_frequency_ = publish_frequency;
    do_publish_ = (publish_frequency > 0.0);
    wall_step_.fromSec(1.0 / publish_frequency);
}
    
void TimeManager::setBagTimeHorizon(ros::Time limit_bag_time)
{
}

void TimeManager::pauseBagTime(bool pause)
{
    if (paused_)
  {
    paused_time_ = ros::WallTime::now();
  }
  else
  {
    // Make sure time doesn't shift after leaving pause.
    ros::WallDuration shift = ros::WallTime::now() - paused_time_;
    paused_time_ = ros::WallTime::now();

    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

    horizon += shift;
    time_publisher_.setWCHorizon(horizon);
  }
}

void TimePublisher::runClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;
      
        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while (t < done && t < wc_horizon_)
        {
            ros::WallDuration leftHorizonWC = wc_horizon_ - t;

            ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
            d *= time_scale_;

            current_ = horizon_ - d;

            if (current_ >= horizon_)
              current_ = horizon_;

            if (t >= next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;
            if (target > wc_horizon_)
              target = wc_horizon_;
            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    } else {

        ros::WallTime t = ros::WallTime::now();

        ros::WallDuration leftHorizonWC = wc_horizon_ - t;

        ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
        d *= time_scale_;

        current_ = horizon_ - d;
        
        if (current_ >= horizon_)
            current_ = horizon_;

        ros::WallTime target = ros::WallTime::now() + duration;

        if (target > wc_horizon_)
            target = wc_horizon_;

        ros::WallTime::sleepUntil(target);
    }
}

void TimePublisher::stepClock(ros::Time const& bag_time, bool shift)
{
    current_ = bag_time;
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;
        pub_msg.clock = current_;
        time_pub_.publish(pub_msg);

        ros::WallTime t = ros::WallTime::now();
        next_pub_ = t + wall_step_;
    }
    if (shift)
    {
    // shift translator?
                    ros::WallDuration shift = ros::WallTime::now() - horizon ;
                    paused_time_ = ros::WallTime::now();

                    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                    horizon += shift;
                    time_publisher_.setWCHorizon(horizon);
    }
}

void TimePublisher::runStalledClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while ( t < done )
        {
            if (t > next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;

            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    } else {
        duration.sleep();
    }
}

bool TimePublisher::horizonReached()
{
  return ros::WallTime::now() > wc_horizon_;
}

} // namespace rosbag
