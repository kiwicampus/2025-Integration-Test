#include <rosbag_player/rosbag_player.hpp>

namespace rosbag_player{

RosBagPlayer::RosBagPlayer(const std::string& rosbag_path) : rosbag_path_(rosbag_path)
{
    // TODO: implement an exception when the file does not exist
}

void RosBagPlayer::play(PlayOptions const& options)
{
    // Start a thread to play the bag file
    
    // TODO: implement the play function
    throw std::logic_error("Not implemented");
}

void RosBagPlayer::stop()
{
    // TODO: Stop the bag player thread
    throw std::logic_error("Not implemented");
}

void RosBagPlayer::join()
{
    //TODO: Wait until the ros bag end
    throw std::logic_error("Not implemented"); 
}
}