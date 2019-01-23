# rs_hsrb_perception
A RoboSherlock package managing the HSRB's perception

## Dependencies
The src folder of your catkin workspace must contain the following packages:
* RoboSherlock (Suturo fork): [https://github.com/Suturo1819/robosherlock](https://github.com/Suturo1819/robosherlock)
* Suturo Msgs: [https://github.com/Suturo1819/suturo_msgs](https://github.com/Suturo1819/suturo_msgs)

Furthermore, you have to install `RoboSherlock` before building this package. To do so, follow the [installation instructions](http://robosherlock.org/install.html) from `RoboSherlock`

## Setup
* Clone this repository to the src folder of your workspace
* Open a terminal in your workspace folder and use `catkin build` (do **NOT** use `catkin_make`)
* Source your workspace

## Run
* In your sourced workspace folder, type `rosrun robosherlock run _ae:=hsrb _vis:=true`
    * The `_vis` tag is optional, if you like to see what the robot sees on your machine
    * The `_ae` tag however is crucial. It specifies the pipeline processing the vision. Use **hsrb** as ae
* Sit back, relax and enjoy your view being processed. The infos are published as `ObjectDetectionData` on the topic */suturo_perception/object_detection*