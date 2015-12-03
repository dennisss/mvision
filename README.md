Monocular Vision Library
========================

- By Dennis Shtatnov <densht@gmail.com>


Papers to read
---------------


- http://www.novatel.com/assets/Documents/Bulletins/APN064.pdf
- https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
- https://github.com/ethz-asl/kalibr/wiki


- Very useful: http://developer.android.com/reference/android/hardware/SensorEvent.html#values


Overview
--------

This project consists of a primary C++ library, `libvio`, which implements common filters for visual inertial odometry:
	- As input, the library takes camera video frames and timestamped accelerometer/gyroscope readings
	- Additionally, certain calibration parameters such as the camera instrinsics and the IMU biases need to be entered
	- In the library, a MSCKF model can be used to find the 6 DOF path of motion (odometry) of the device
		- This part is designed to run in realtime on a single core of a smartphone CPU without GPU acceleration
	- There are also portions of the library for maintaining a sparse map




For now, in order to determine the initial orientation, the camera needs to be help still at the start so that the orientation can be estimated from the raw steady acceleration readings


Also, in the `android/` folder is an Android app showing its use on a smartphone.
	- The Android version requires the NDK and has only been tested on a Nexus 6



TODO: Allow multiple cameras
TODO: Use vanishing directions combined with gravity to get initial (and regular) orientation updates
TODO: Allow use of compass, GPS, barometer, etc.

TODO: Allow camera calibration from within the app

TODO: For Snapdragon/Android builds, the image processing code should use Qualcomm's FastCV library


For running VisualSFM
---------------------

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/dennis/software/vsfm/bin ./VisualSFM sfm+k=1630.8706287301968,982.51095058012686,1626.9354701674479,679.82201075835394+sort ~/data/MVision/walkway/imgs/ ~/data/MVision/walkway/phone.nvm

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/dennis/software/vsfm/bin ./VisualSFM sfm+loadnvm+pmvs+sort ~/data/MVision/walkway/phone.nvm ~/data/MVision/walkway/pmvs.nvm




Calibration
-----------

- Camera intrinsics can be calibrated using standard OpenCV scripts

- Holding the IMU still should generate a normal distribution of reading that can be used to derive






- Multiple different threads should be running

	- One should be doing odometry tracking semi-dense edges

	- Mapping should eventually simplify feature points to planes that can be refined and potentially used for loop closures, especially at plane intersection points

		- The gravity vector can be used to determine where the floor should be and it can be marked
