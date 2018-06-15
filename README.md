[//]: # (Image References)

[img1]: ./images/finished.png "finished.png"
[img2]: ./images/Meas_vs_Kalmanfilt.png "measvskalman.png"
[img3]: ./images/NIS_process_noise_bad.png "finished.png"
[img4]: ./images/NIS_process_noise_good.png "finished.png"

---
# SDCND Term 2 Project 7: Unscented Kalman Filter
## Project for Udacity Self-Driving Car Engineer Nanodegree Program

In this project we will utilize a Unscented Kalman Filter [UKF] to estimate the state of a moving object of interest with noisy lidar and radar measurements. It's similiar to the project ["Extended Kalman Filter"](https://github.com/autonomobil/SDCND-P6_Extended-Kalman-Filter)

This project includes  the implementation of an Unscented Kalman filter with C++. A Udacity-provided simulator (available for download [here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of an object's position and speed, and the UKF must merge these measurements to predict the object's position. Communication between the simulator and the EKF takes place via [uWebSocket](https://github.com/uNetworking/uWebSockets).

Udacity's project basis can be found [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).

![img2]


## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator

For instructions on how to install these components on different operating systems, visit [Udacity's project](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).


## Setup and Running
These are the suggested steps for Windows setup:

* Follow these [instructions](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up Ubuntu BASH.
* Download Windows simulator [here](https://github.com/udacity/self-driving-car-sim/releases).
* Open Ubuntu Bash (write following commands to Ubuntu Bash command window)
* ``sudo apt-get update``
* ``sudo apt-get install git``
* ``sudo apt-get install cmake``
* ``sudo apt-get install openssl``
* ``sudo apt-get install libssl-dev``
* navigate to where you want to clone this repository to, for example:
 ``cd /mnt/c/Users/Bob``
* ``git clone https://github.com/autonomobil/SDCND-P7_Unscented-Kalman-Filter``
* ``sudo rm /usr/lib/libuWS.so``
* navigate to project folder: ``cd SDCND-P7_Unscented-Kalman-Filter``
* ``./install-ubuntu.sh``
* ``mkdir build && cd build``
* ``cmake .. && make``
* Launch the **term2_sim.exe** from Windows simulator folder
* Execute ``./UnscentedKF``
* If you see ``Listening to port 4567 Connected!!!``, it is working
* Press **Start**

These files were modified compared to the [original repository](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project):  
* src/ukf.cpp
* src/ukf.h
* src/tools.cpp

An output data file``data_out.txt``  will be created in ``./build``, which consists of ``'px_meas','py_meas','NIS_radar', 'NIS_laser', 'px', 'py'``.

There is also a python routine ([UKF_Visualizer.ipynb](./UKF_Visualizer.ipynb)) implemented to show measured and predicted points, as well as NIST values to tune the process noise ``std_a`` and ``std_yawdd``. These were tuned to ``std_a = 1`` and ``std_yawdd = 0.3``.

**Not so good NIST values:**
![img3]
**Better NIST values:**
![img4]

---

## Checking Rubric points
#### Your code should compile.
* The code compiles without errors on my setup following the instructions above.

#### px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30]  when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.
* Dataset 1 RMSE :
  * [0.0733, 0.0811, 0.1768, 0.1665] with initializing ``x_(2) = 5;``
    * doesn't work if you start in a different direction; dataset 2: ``x_(2)`` would have to be ``-5``
  * [0.0700, 0.0833, 0.3406, 0.2271] with initializing ``if(radar)  {x_(2) = sqrt(vx * vx + vy * vy);} else{x_(2) = 0;}``(pseudo-code)
    * does work in all directions, so dataset 2 is possible


* Dataset 2 RMSE : [0.0685, 0.0682, 0.4186, 0.2357]


![img1]


#### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons
* Sensor Fusion  @ [./src/ukd.cpp](./src/ukd.cpp)
* [./src/ukd.cpp](./src/ukd.cpp) uses functions``Prediction``, ``UpdateLidar``, ``UpdateRadar`` and ``Update_x_P``


#### Your Kalman Filter algorithm handles the first measurements appropriately
* First measurement and initialization of x and P is handled in lines 100 - 151 [./src/ukd.cpp](./src/ukd.cpp)


#### Your Kalman Filter algorithm first predicts then updates
* Prediction function: Lines 202 -287 [./src/ukd.cpp](./src/ukd.cpp); function `Prediction`
* Update functions: Lines 307 - 440 [./src/ukd.cpp](./src/ukd.cpp);
functions ``UpdateLidar``, ``UpdateRadar`` and ``Update_x_P``

#### Your Kalman Filter can handle radar and lidar measurements
* In [./src/ukd.cpp](./src/ukd.cpp) if-statements decide how to initialize and process the data given the sensor type
