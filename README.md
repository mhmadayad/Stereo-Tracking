## OpenCV C++ Stereo Tracking System

Tracking of a needle (Opthalmic Surgery) using 3 balls marker 

### Dependencies

- OpenCV
- popt

### Compilation

mkdir build && cd build
cmake ..
make all


### Get images from webcams

This is a small helper tool to grab frames from two webcams operating as a stereo pair. Run the following command to use it.


./read -w [img_width] -h [img_height] -d [imgs_directory] -e [file_extension]


Images are saved with prefixes `left` and `right` in the desired directory.

### Intrinsic calibration of a single camera

./calibrate -w [board_width] -h [board_height] -n [num_imgs] -s [square_size](in cm) -d [imgs_directory] -i [imgs_filename] -o [file_extension] -e [output_filename]

./calibrate -w 9 -h 6 -n 27 -s 0.0333 -d "../calib_imgs/1/" -i "left" -o "cam_left.yml" -e "jpg"


### Stereo calibration for extrinisics

Once you have the intrinsics calibrated for both the left and the right cameras, you can use their intrinsics to calibrate the extrinsics between them.

./calibrate_stereo -n [num_imgs] -u [left_cam_calib] -v [right_cam_calib] -L [left_img_dir] -R [right_img_dir] -l [left_img_prefix] -r [right_img_prefix] -o [output_calib_file]

./calibrate_stereo -n 27 -u cam_left.yml -v cam_right.yml -L ../calib_imgs/1/ -R ../calib_imgs/1/ -l left -r right -o cam_stereo.yml


### Undistortion and Rectification

Once you have the stereo calibration data, you can remove the distortion and rectify any pair of images so that the resultant epipolar lines become scan lines.

./undistort_rectify -l [left_img_path] -r [right_img_path] -c [stereo_calib_file] -L [output_left_img] -R [output_right_img]

./undistort_rectify -l ../calib_imgs/1/left1.jpg -r ../calib_imgs/1/right1.jpg -c cam_stereo.yml -L left.jpg -R right.jpg


## Circle Detection
Detection of balls (using cv::findContours) based on pre-defined min - max radius which can be adjusted from Constants.h
We already know the setup of the balls , we can create a triangle out of them-> tracking follows easily since we know the distance between the centers of the balls.
There is no tracking algorithm will work here since we have 3 same balls,  and they are textureless , edgeless. So we cant extract any features for tracking.

already tracking is done but in case we lost the detection in anyframe we should keep a tracking algorithm which can predict the location of the ball in the next frame.
TODO::Tracking alogrithms ->
1-Lukas Kanade
2-Kalman filter
