#ifndef ROSBAG_PLAYER__ROSBAG_PLAYER_HPP_
#define ROSBAG_PLAYER__ROSBAG_PLAYER_HPP_

#include <pthread.h>
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rosbag_player{

// Modify tis struct if you want
struct PlayOptions{
    float start_time;                // start time offset in seconds
    float duration;                  // play duration in seconds
    std::vector<std::string> topics; // topics to publish
};

class RosBagPlayer
{
    typedef void* (*THREADFUNCPTR)(void*);

   public:
    RosBagPlayer(const std::string& rosbag_path);

    /*!
    Start a thread to play the bag file
    */
    void play();

    /*!
    stop bag play
    */
    void stop();

    /*!
    join bag play thread process
    */
    void join();

   private:
    std::string rosbag_path_;
    std::atomic<bool> stop_flag_;
    pthread_t bag_player_;

    /*!
    Play the ros bag in a second thread
    */
    void play_bag_file();
};
} // rosbag_player

#endif  // ROSBAG_PLAYER__ROSBAG_PLAYER_HPP_