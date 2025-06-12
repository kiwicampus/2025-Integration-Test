from typing import List


class RosBagPlayer:
    def __init__(self, rosbag_path: str):
        """! Create RosBagPlayer object
        @param gcs_path (str) GCP path were the rosbag is stored

        Raises:
            RuntimeError: File was not found
        """
        self.local_path = rosbag_path
        # TODO:
        # implement the exception when the file does not exist

    def play(self, topics: List[str], start_time: float, duration: float):
        # Start a thread to play the bag file
        # topics: topics to publish
        # start_time: start time offset in seconds
        # duration: play duration in seconds

        # TODO: implement the play function, define default values if you want
        raise NotImplementedError()

    def join(self):
        # TODO:
        # Wait until the ros bag end
        raise NotImplementedError()

    def stop(self):
        # TODO:
        # Stop the bag player thread
        raise NotImplementedError()
