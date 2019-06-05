/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
********************************************************************/

#include "rosbag/player.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"

#if !defined(_MSC_VER)
  #include <sys/select.h>
#endif

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "rosgraph_msgs/Clock.h"

#define foreach BOOST_FOREACH

using std::map;
using std::pair;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Exception;

namespace rosbag {

ros::AdvertiseOptions createAdvertiseOptions(const ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    ros::M_string::const_iterator header_iter = c->header->find("latching");
    opts.latch = (header_iter != c->header->end() && header_iter->second == "1");
    return opts;
}


ros::AdvertiseOptions createAdvertiseOptions(MessageInstance const& m, uint32_t queue_size, const std::string& prefix) {
    return ros::AdvertiseOptions(prefix + m.getTopic(), queue_size, m.getMD5Sum(), m.getDataType(), m.getMessageDefinition());
}

// PlayerOptions

PlayerOptions::PlayerOptions() :
    prefix(""),
    quiet(false),
    start_paused(false),
    at_once(false),
    bag_time(false),
    bag_time_frequency(0.0),
    time_scale(1.0),
    queue_size(0),
    advertise_sleep(0.2),
    try_future(false),
    has_time(false),
    loop(false),
    time(0.0f),
    has_duration(false),
    duration(0.0f),
    keep_alive(false),
    wait_for_subscribers(false),
    rate_control_topic(""),
    rate_control_max_delay(1.0f),
    skip_empty(ros::DURATION_MAX)
{
}

void PlayerOptions::check() {
    if (bags.size() == 0)
        throw Exception("You must specify at least one bag file to play from");
    if (has_duration && duration <= 0.0)
        throw Exception("Invalid duration, must be > 0.0");
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options),
    delayed_(false),
    // If we were given a list of topics to pause on, then go into that mode
    // by default (it can be toggled later via 't' from the keyboard).
    pause_for_topics_(options_.pause_topics.size() > 0),
    pause_change_requested_(false),
    requested_pause_state_(false),
    terminal_modified_(false)
{
  ros::NodeHandle private_node_handle("~");
  pause_service_ = private_node_handle.advertiseService("pause_playback", &Player::pauseCallback, this);
}

Player::~Player() {
    foreach(shared_ptr<Bag> bag, bags_)
        bag->close();

    restoreTerminal();
}

void Player::publish() {
    options_.check();

    // Open all the bag files
    foreach(string const& filename, options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());

        try
        {
            shared_ptr<Bag> bag(boost::make_shared<Bag>());
            bag->open(filename, bagmode::Read);
            bags_.push_back(bag);
        }
        catch (BagUnindexedException ex) {
            std::cerr << "Bag file " << filename << " is unindexed.  Run rosbag reindex." << std::endl;
            return;
        }
    }

    setupTerminal();

    if (!node_handle_.ok())
      return;

    if (!options_.prefix.empty())
    {
      ROS_INFO_STREAM("Using prefix '" << options_.prefix << "'' for topics ");
    }

    if (!options_.quiet)
      puts("");
    
    // Publish all messages in the bags
    View full_view;
    foreach(shared_ptr<Bag> bag, bags_)
        full_view.addQuery(*bag);

    ros::Time initial_time = full_view.getBeginTime();

    initial_time += ros::Duration(options_.time);

    ros::Time finish_time = ros::TIME_MAX;
    if (options_.has_duration)
    {
      finish_time = initial_time + ros::Duration(options_.duration);
    }

    View view;
    TopicQuery topics(options_.topics);

    if (options_.topics.empty())
    {
      foreach(shared_ptr<Bag> bag, bags_)
        view.addQuery(*bag, initial_time, finish_time);
    } else {
      foreach(shared_ptr<Bag> bag, bags_)
        view.addQuery(*bag, topics, initial_time, finish_time);
    }

    if (view.size() == 0)
    {
      std::cerr << "No messages to play on specified topics.  Exiting." << std::endl;
      ros::shutdown();
      return;
    }

    // Advertise all of our messages
    foreach(const ConnectionInfo* c, view.getConnections())
    {
        ros::M_string::const_iterator header_iter = c->header->find("callerid");
        std::string callerid = (header_iter != c->header->end() ? header_iter->second : string(""));

        string callerid_topic = callerid + c->topic;

        map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
        if (pub_iter == publishers_.end()) {

            ros::AdvertiseOptions opts = createAdvertiseOptions(c, options_.queue_size, options_.prefix);

            ros::Publisher pub = node_handle_.advertise(opts);
            publishers_.insert(publishers_.begin(), pair<string, ros::Publisher>(callerid_topic, pub));

            pub_iter = publishers_.find(callerid_topic);
        }
    }

    if (options_.rate_control_topic != "")
    {
        std::cout << "Creating rate control topic subscriber..." << std::flush;

        boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
        ros::SubscribeOptions ops;
        ops.topic = options_.rate_control_topic;
        ops.queue_size = 10;
        ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
        ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
        ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
            const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                boost::bind(&Player::updateRateTopicTime, this, _1));

        rate_control_sub_ = node_handle_.subscribe(ops);

        std::cout << " done." << std::endl;
    }


    std::cout << "Waiting " << options_.advertise_sleep.toSec() << " seconds after advertising topics..." << std::flush;
    options_.advertise_sleep.sleep();
    std::cout << " done." << std::endl;

    std::cout << std::endl << "Hit space to toggle paused, or 's' to step." << std::endl;

    if (options_.wait_for_subscribers)
    {
        waitForSubscribers();
    }

    time_publisher_.pauseClock(options_.start_paused);

    while (true) {
        // Set up our time publishers

        start_time_ = view.begin()->getTime();
        bag_length_ = view.getEndTime() - view.getBeginTime();

        // Set the last rate control to now, so the program doesn't start delayed.
        last_rate_control_ = start_time_;

        time_publisher_.setTime(start_time_);

        time_publisher_.setTimeScale(options_.time_scale);
        if (options_.bag_time)
            time_publisher_.setPublishFrequency(options_.bag_time_frequency);
        else
            time_publisher_.setPublishFrequency(-1.0);

        // Call do-publish for each message
        foreach(MessageInstance m, view) {
            if (!node_handle_.ok())
                break;

            doPublish(m);
        }

        if (options_.keep_alive)
            while (node_handle_.ok())
                doKeepAlive();

        if (!node_handle_.ok()) {
            std::cout << std::endl;
            break;
        }
        if (!options_.loop) {
            std::cout << std::endl << "Done." << std::endl;
            break;
        }
    }

    ros::shutdown();
}

void Player::updateRateTopicTime(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event)
{
    boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = msg_event.getConstMessage();
    std::string def = ssmsg->getMessageDefinition();
    size_t length = ros::serialization::serializationLength(*ssmsg);
    
    // Check the message definition.
    std::istringstream f(def);
    std::string s;
    bool flag = false;
    while(std::getline(f, s, '\n')) {
        if (!s.empty() && s.find("#") != 0) {
            // Does not start with #, is not a comment.
            if (s.find("Header ") == 0) {
                flag = true;
            }
            break;
        }
    }
    // If the header is not the first element in the message according to the definition, throw an error.
    if (!flag) {
        std::cout << std::endl << "WARNING: Rate control topic is bad, header is not first. MSG may be malformed." << std::endl;
        return;
    }

    std::vector<uint8_t> buffer(length);
    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *ssmsg);

    // Assuming that the header is the first several bytes of the message.
    //uint32_t header_sequence_id   = buffer[0] | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 24;
    int32_t header_timestamp_sec  = buffer[4] | (uint32_t)buffer[5] << 8 | (uint32_t)buffer[6] << 16 | (uint32_t)buffer[7] << 24;
    int32_t header_timestamp_nsec = buffer[8] | (uint32_t)buffer[9] << 8 | (uint32_t)buffer[10] << 16 | (uint32_t)buffer[11] << 24;

    last_rate_control_ = ros::Time(header_timestamp_sec, header_timestamp_nsec);
}

void Player::printTime()
{
    if (!options_.quiet) {

        ros::Time current_time = time_publisher_.getTime();
        ros::Duration d = current_time - start_time_;


        if (delayed_)
        {
            ros::Duration time_since_rate = std::max(ros::Time::now() - last_rate_control_, ros::Duration(0));
            printf("\r [DELAYED]  Bag Time: %13.6f   Duration: %.6f / %.6f   Delay: %.2f \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec(), time_since_rate.toSec());
        }
        else if (time_publisher_.isPaused())
        {
            printf("\r [PAUSED ]  Bag Time: %13.6f   Duration: %.6f / %.6f               \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec());
        }
        else
        {
            printf("\r [RUNNING]  Bag Time: %13.6f   Duration: %.6f / %.6f               \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec());
        }
        fflush(stdout);
    }
}

bool Player::pauseCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  pause_change_requested_ = (req.data != time_publisher_.isPaused());
  requested_pause_state_ = req.data;

  res.success = pause_change_requested_;

  if (res.success)
  {
    res.message = std::string("Playback is now ") + (requested_pause_state_ ? "paused" : "resumed");
  }
  else
  {
    res.message = std::string("Bag is already ") + (requested_pause_state_ ? "paused." : "running.");
  }

  return true;
}

void Player::waitForSubscribers() const
{
    bool all_topics_subscribed = false;
    std::cout << "Waiting for subscribers." << std::endl;
    while (!all_topics_subscribed) {
        all_topics_subscribed = true;
        foreach(const PublisherMap::value_type& pub, publishers_) {
            all_topics_subscribed &= pub.second.getNumSubscribers() > 0;
        }
        ros::Duration(0.1).sleep();
    }
    std::cout << "Finished waiting for subscribers." << std::endl;
}

void Player::doPublish(MessageInstance const& m) {
    string const& topic   = m.getTopic();
    ros::Time const& time = m.getTime();
    string callerid       = m.getCallerId();
    
    string callerid_topic = callerid + topic;

    map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
    ROS_ASSERT(pub_iter != publishers_.end());

    // Update subscribers.
    ros::spinOnce();

    // If immediate specified, play immediately
    if (options_.at_once) {
        time_publisher_.stepClock(time);
        pub_iter->second.publish(m);
        printTime();
        return;
    }

    // If skip_empty is specified, skip this region and shift.
    if (time - time_publisher_.getTime() > options_.skip_empty)
    {
      time_publisher_.stepClock(time);

      (pub_iter->second).publish(m);
      printTime();
      return;
    }

    if (pause_for_topics_)
    {
        for (std::vector<std::string>::iterator i = options_.pause_topics.begin();
             i != options_.pause_topics.end();
             ++i)
        {
            if (topic == *i)
            {
                time_publisher_.pauseClock(true);
            }
        }
    }

    // Check if the rate control topic has posted recently enough to continue, or if a delay is needed.
    // Delayed is separated from paused to allow more verbose printing.
    if (rate_control_sub_ != NULL) {
        if ((time_publisher_.getTime() - last_rate_control_).toSec() > options_.rate_control_max_delay) {
            delayed_ = true;
            time_publisher_.pauseClock(true);
        }
    }

    while ((time_publisher_.isPaused() || time_publisher_.getTime() < time) && node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_.ok())
        {
            ros::spinOnce();

            if (pause_change_requested_)
            {
                time_publisher_.pauseClock(requested_pause_state_);
                pause_change_requested_ = false;
            }

            switch (readCharFromStdin()){
            case ' ':
                time_publisher_.pauseClock(!time_publisher_.isPaused());
                break;
            case 's':
                if (time_publisher_.isPaused()) {
                    time_publisher_.stepClock(time);

                    (pub_iter->second).publish(m);

                    printTime();
                    return;
                }
                break;
            case 't':
                pause_for_topics_ = !pause_for_topics_;
                break;
            case EOF:
                if (time_publisher_.isPaused())
                {
                    printTime();
                    time_publisher_.runStalledClock(ros::WallDuration(.1));
                    ros::spinOnce();
                    if (delayed_)
                    {
                        // You need to check the rate here too.
                        if(rate_control_sub_ == NULL || (time_publisher_.getTime() - last_rate_control_).toSec() <= options_.rate_control_max_delay) {
                            time_publisher_.pauseClock(false);
                            delayed_ = false;
                        }
                    }
                }
                else
                    charsleftorpaused = false;
            }  // switch
        }  // while
        printTime();
        time_publisher_.runClock(ros::WallDuration(.1), time);
        ros::spinOnce();
    }  // while

    pub_iter->second.publish(m);
}


void Player::doKeepAlive() {
    //Keep pushing ourself out in 10-sec increments (avoids fancy math dealing with the end of time)
    ros::Time const& time = time_publisher_.getTime() + ros::Duration(10.0);

    if (options_.at_once) {
        return;
    }

    // If we're done and just staying alive, don't watch the rate control topic. We aren't publishing anyway.
    delayed_ = false;

    while ((time_publisher_.isPaused() || time_publisher_.getTime() < time) && node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_.ok())
        {
            switch (readCharFromStdin()){
            case ' ':
                time_publisher_.pauseClock(!time_publisher_.isPaused());
                break;
            case EOF:
                if (time_publisher_.isPaused())
                {
                    printTime();
                    time_publisher_.runStalledClock(ros::WallDuration(.1));
                    ros::spinOnce();
                }
                else
                    charsleftorpaused = false;
            }
        }

        printTime();
        time_publisher_.runClock(ros::WallDuration(.1), time);
        ros::spinOnce();
    }
}



void Player::setupTerminal() {
    if (terminal_modified_)
        return;

#if defined(_MSC_VER)
    input_handle = GetStdHandle(STD_INPUT_HANDLE);
    if (input_handle == INVALID_HANDLE_VALUE)
    {
        std::cout << "Failed to set up standard input handle." << std::endl;
        return;
    }
    if (! GetConsoleMode(input_handle, &stdin_set) )
    {
        std::cout << "Failed to save the console mode." << std::endl;
        return;
    }
    // don't actually need anything but the default, alternatively try this
    //DWORD event_mode = ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT;
    //if (! SetConsoleMode(input_handle, event_mode) )
    //{
    // std::cout << "Failed to set the console mode." << std::endl;
    // return;
    //}
    terminal_modified_ = true;
#else
    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);

    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
    terminal_modified_ = true;
#endif
}

void Player::restoreTerminal() {
	if (!terminal_modified_)
		return;

#if defined(_MSC_VER)
    SetConsoleMode(input_handle, stdin_set);
#else
    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);
#endif
    terminal_modified_ = false;
}

int Player::readCharFromStdin() {
#ifdef __APPLE__
    fd_set testfd;
    FD_COPY(&stdin_fdset_, &testfd);
#elif !defined(_MSC_VER)
    fd_set testfd = stdin_fdset_;
#endif

#if defined(_MSC_VER)
    DWORD events = 0;
    INPUT_RECORD input_record[1];
    DWORD input_size = 1;
    BOOL b = GetNumberOfConsoleInputEvents(input_handle, &events);
    if (b && events > 0)
    {
        b = ReadConsoleInput(input_handle, input_record, input_size, &events);
        if (b)
        {
            for (unsigned int i = 0; i < events; ++i)
            {
                if (input_record[i].EventType & KEY_EVENT & input_record[i].Event.KeyEvent.bKeyDown)
                {
                    CHAR ch = input_record[i].Event.KeyEvent.uChar.AsciiChar;
                    return ch;
                }
            }
        }
    }
    return EOF;
#else
    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
        return EOF;
    return getc(stdin);
#endif
}

TimePublisher::TimePublisher() : time_scale_(1.0)
{
  setPublishFrequency(-1.0);
  time_pub_ = node_handle_.advertise<rosgraph_msgs::Clock>("clock",1);
  is_paused_ = false;
  last_run_wall_time_ = ros::WallTime::now();
}

void TimePublisher::setPublishFrequency(double publish_frequency)
{
  publish_frequency_ = publish_frequency;
  
  do_publish_ = (publish_frequency > 0.0);

  wall_step_.fromSec(1.0 / publish_frequency);
}

void TimePublisher::setTimeScale(double time_scale)
{
    time_scale_ = time_scale;
}

double TimePublisher::getTimeScale()
{
    return time_scale_;
}

void TimePublisher::setTime(const ros::Time& time)
{
    current_ = time;
}

ros::Time const& TimePublisher::getTime() const
{
    return current_;
}

bool TimePublisher::isPaused()
{
    return is_paused_;
}

void TimePublisher::pauseClock(bool pause)
{
    ros::WallTime now = ros::WallTime::now();
    if (!pause && is_paused_)
    {
        last_run_wall_time_ = now;
    }

    is_paused_ = pause;
}

void TimePublisher::runClock(const ros::WallDuration& duration, const ros::Time& horizon)
{
    // Update ros time with how much wall-clock time has passed since the last
    // time we ran the clock. Otherwise, at high time-scales, (or when we have
    // lots of messages coming in quickly) we can end up sleeping much longer
    // than we should because a larger percentage of our time is spent _not_
    // sleeping.
    current_ += toRosDuration(ros::WallTime::now() - last_run_wall_time_);

    ros::WallDuration remaining_duration = duration;
    if (current_ + toRosDuration(remaining_duration) > horizon)
    {
        remaining_duration = toWallDuration(horizon - current_) + ros::WallDuration(0, 10);
    }

    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;
        while (remaining_duration > ros::WallDuration(0))
        {
            ros::WallTime t = ros::WallTime::now();
            if (t >= next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }
            ros::WallDuration wc_duration = remaining_duration;
            if (t + wc_duration > next_pub_)
            {
                wc_duration = next_pub_ - t;
            }
            if (wc_duration > ros::WallDuration(0))
            {
                ros::WallTime::sleepUntil(t + wc_duration);
                current_ += toRosDuration(wc_duration);
                remaining_duration -= wc_duration;
            }
        }
        // Make sure we publish time regardless of what duration was requested.
        ros::WallTime t = ros::WallTime::now();
        if (t >= next_pub_)
        {
            pub_msg.clock = current_;
            time_pub_.publish(pub_msg);
            next_pub_ = t + wall_step_;
        }
    } else {
        ros::WallTime t = ros::WallTime::now();
        ros::WallDuration wc_duration = duration;
        if (wc_duration > ros::WallDuration(0))
        {
            ros::WallTime::sleepUntil(t + wc_duration);
            current_ += toRosDuration(wc_duration);
        }
    }

    last_run_wall_time_ = ros::WallTime::now();
}

void TimePublisher::stepClock(const ros::Time& horizon)
{
    if (do_publish_)
    {
        current_ = horizon;

        rosgraph_msgs::Clock pub_msg;

        pub_msg.clock = current_;
        time_pub_.publish(pub_msg);

        ros::WallTime t = ros::WallTime::now();
        next_pub_ = t + wall_step_;
    } else {
        current_ = horizon;
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

ros::Duration TimePublisher::toRosDuration(const ros::WallDuration& duration)
{
    return ros::Duration(duration.toSec() * time_scale_);
}

ros::WallDuration TimePublisher::toWallDuration(const ros::Duration& duration)
{
    return ros::WallDuration(duration.toSec() / time_scale_);
}

} // namespace rosbag
