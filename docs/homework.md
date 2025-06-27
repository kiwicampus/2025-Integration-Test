<!-- ---------------------------------------------------------------------- -->

---
# FINAL PROJECT
NAME: **WRITE YOUR NAME HERE**

---
**INSTRUCTIONS**:
1. Clone this repository, and associate it with your GitHub account and make it private for you to make pushes without allowing other participants to see your answers. You'll be asked to make it public the due date. NOT before!
2. Create a new branch name as `develop` and work all your solutions there.
3. Every time that you complete a basic point create a commit, add the files and push with the next format description: `[FEAT]: description of what you did`. See the [conventions.md](./conventions.md) file for more details.
4. You can do your last commit and push until and before the deadline (date & time) if you do more after, we will make a reverse of your code until the last commit before the dead-line date-time (If the project is not complete there, it'll be rejected).
5. Remember partial solutions will be no accepted, you have to complete at least **BASIC POINTS**, **Questionnaire**, but we consider the real project the **EXTRA-HOMEWORK** session (Without it your chances will be less).
6. Rosbag's are located in [Drive](https://drive.google.com/file/d/1du7JgN_haaAiAGsxCDTqvEmQIG1qtHxE/view?usp=sharing) please download them and save them into the [rosbags folder](/rosbags). You should source `. /opt/ros/iron/setup.bash` and `. /workspace/rover/ros2/install/setup.bash` in order to be able to check all the topics. 

> Note: the last one is generated once you build `usr_msgs` package

--------
**BASIC POINTS**:

Here are the basic points to accomplish the project, what we are evaluating you is the knowledge in Docker, programming languages, ROS2 in a basic level of concepts and implementation, logical and algorithms. This part of the project is more for the understanding of itself because we invite you to make the extra-homework, which is the real challenge, where you'll have to be more creative and go further. 

Well, the recommendation in this section is that you won't have to change the input or output attributes in any function or method, just implement functions contents, and operations, import a library and use it inside (this rule doesn't apply in the extra-homework points).

Try first to understand the function of the node in the whole stack, then what is every function for, read the documentation and docs provided, and look for the TODO sections in these.


Running the stack from the root, remember the command is:

```
koda@airobotics:/workspace$ bash scripts/startRobotics.sh
```

## *Docker*

1. You are fresh here, going to start your project and oh no! The docker image is broken! This is something that you may have to encounter in your day-to-day work, so let's fix it. Refer to the [Dockerfile](/.devcontainer/Dockerfile) to fix the docker image.
2. Ok, now you are able to use the dev-container! but you are suppose to source `. /opt/ros/iron/setup.bash` and `. /workspace/rover/ros2/install/setup.bash` each time you open a terminal, so annoying; add those instructions in the `.bashrc` file so they will source automatically from the Dockerfile.


## *Debug and diagnose*

One of the main habilities that you should have is to debug and diagnose the system. Some times the robot has some unexpected behaviors, or edge cases we need to consider. In order to solve this kind of issues, you should know how to use the tools that ROS2 provides you.

We are providing you several [rosbags](https://drive.google.com/drive/u/0/folders/1xMR8ObD-Jt19MIGLF3YgpQJIerUfpXvC) with different scenarios that you should analyze and understand what is happening in each one of them.

**The input topics will be supplied by the `rosbag`'s. Such as:**

- `/imu/data`
- `/camera/imu`
- `/camera/color/camera_info `
- `/wheel_odometry/local_odometry`
- `/motion_control/speed_controller/reference_cmd`
- `/livox/lidar`
- `/local_plan`


Now, what easy way to analyze the topics? Kiwibot is here for you! We developed a tool that will help you to visualize the topics and understand what is happening in the robot using [rosboard and foxglove.](https://discourse.ros.org/t/introducing-foxglove-integration-with-rosboard-for-real-time-visualizations/38376). Don't worry we have already installed it for you in the dev-container. Check the rosboard and rosboard_client packages.To run the tool you just need to enable the launch in the [nodes_launch.sh](/rover/configs/nodes_launch.sh) file.

  1. Identify the `NODE_ROSBOARD` on the [nodes_launch.sh](/rover/configs/nodes_launch.sh) file: 
      ```.sh
      export NODE_ROSBOARD=1 # rosboard skip:1
      ```

  2. When you run the node a pop-up will appear, click on the `Open in browser` button.
  3. Open the foxglove tab and locally play the rosbag you are interested in.
  4. Navigate through the topics and try to understand the data.

  5. Answer the following questions using the diagnostic_0.mcap rosbag you have:

  Our team-mates reported something weird about a robot. According to the robotics-supervisor, the robot just stopped and did not want to continue navigating, even though it moved without control and almost chased. They provided you with the necessary rosbag with the topics to debug the problem and give them a quick response. They need to know: 

  - Was it a real problem? 
  - Who was the murderer? 
  - If it was a real problem, what do you think should be the next step to debug it more deeply, or what could be the solution to avoid it in the future?
  - I heard that you will implement a new fail_detection node. Can you run it with this data and tell me if it can detect the collision? I am not sure if it is crashed or not.
  
## *C++ and/or Python* 

### Fail Detection

First, you need to modify [nodes_launch.sh](/rover/configs/nodes_launch.sh) file to build the `fail_detection` node you are going to implement. Set the `NODE_FAIL_DETECTION_...` to 

1.  Identify the `NODE_FAIL_DETECTION_...` on the [nodes_launch.sh](/rover/configs/nodes_launch.sh) file: 

  ```.sh
  export NODE_FAIL_DETECTION_CPP=0 # fail_detection_cpp skip:0
  export NODE_FAIL_DETECTION_PY=0 # fail_detection_py skip:0
  ```

  You are free to chose if work with the c++ or the python version, but cpp option include a bonus in your score! 🎁

2. Implement a collition detector in your `fail_detection` node. The provided skeleton already implements all subscribers you will need. Publish the accident in the topic `/fail_detection/fail`
3. rosbags `rosx.mcap_x.mcap` have real cases of collitions, use them to test your solution.

---
**REMEMBER**: 
1. If you push just one minute after the time given by email we won't review your project solution (at least, that commit).
2. If you find an error in the code, bug, drawback, and there's no already an issue created in the main repository, please create it, and you win extra points (+3%/5: **this is 3% more over 5.0, which is equal to 0.15 in the final project's grade**), we'll try to give a solution ASAP.
3. You have 3 questions that can be done as an issue in the main repository, so please check that what you are asking for is not already answered in the issues section (don't waste your questions).
4. It's forbidden talk with other project participants, if we think that you cheated, or you cheated, your project and the others (if apply) won't be reviewed and your application will be canceled.
5. What we are evaluating is:
    * **[5%/5]** Code style
    * **[5%/5]** Code Documentation
    * **[15%/5]** Questionnaire
    * **[25%/5]** Solution to basic home-work
    * **[50%/5]** Solution review & answers
6. You can have a grade higher than 5.0.
7. We'll share the final grades with every candidate by email, with feedback included of his/her application (things to improve, to keep, to remove).
8. We'll select the 2 best candidates by their grades, and we'll make the final decision with the Ai&Robotics Team.
9. We are evaluating your concepts and knowledge in ROS2, Python, C++, other Code stuff.
10. If no participant send the project before the deadline time, we will extend the deadline date and time for everyone. However, just the people that complete the questionnaire and the basic-points the rest are doomed. Also, if we considered that the solutions received are not enough or not well explained we also can extend the deadline, but we are pretty sure that this won't happen.

---
<!-- ---------------------------------------------------------------------- -->
## **Questionnaire**

Respond below in the same solution branch every question. In case your answer isn't in this file, it'll not be valid:

1. [C++] Why use an structure for the play options?

1. [ROS] What is de dference between the transient local and the system_default QoS durability?

1. [Docker] Explain with your own words what is the instructions `apt-get autoremove && apt-get clean -y for`?

1. [Docker] If you modify a layer what happen with the previous and the next ones?

1. [Docker] Can we change the basic image (`FROM ubuntu:22.04`) from the docker file to another?


Next questions are after you finish the project, it doesn't give points, but we really appreciate you feedback:
* What do you think about this project? Is it hard or enough? 
* Is it well structured, explanations are clear?

---
<!-- ---------------------------------------------------------------------- -->
## **EXTRA-HOMEWORK**

For extra homework you should create a new branch from the developed one when you finish the basic points of the homework, you can name this new branch as *feature/extra_homework*, don't forget to push it before the time that it was given.

1. **[+4%/5.0]**: Modify the [alias_script.sh](/scripts/alias_script.sh) file so that the `stack-build-` commands have autocompletion.
2. **[+16%/5.0]:** Create a Dockerfile to build OpenCV from scratch based on the `ubuntu:20.04` public image
3. **[+6%/5.0]:** Implement the second fail_detection node in the language you didn't select before.
4. **[+6%/5.0]**: Add the `DELETE_BUILD` argument to the `startRobotics.sh` file. Options : `--delete-build / -d`. Example: `startRobotics.sh -d 1` will delete the previous generated install, build, and log folders. Use 0 as default value.
5. **[+10%/5.0]**: Implements the rosbag_player
5. **[+40%/5.0]**: Implement unit test for your fail detector node, it must be executed using `colcon test`, use the rosbags to feed the node subscribers.

---

<p align="center">
  <img height="300" src="https://user-images.githubusercontent.com/39452483/170424558-2efbe421-727f-4825-9d29-16118c1cacf6.gif">
</p>

