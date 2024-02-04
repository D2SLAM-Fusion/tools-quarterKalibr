# quarterKalibr
> calibrate quadcam modules in a more effecient way
## Background
Calirabte quadcam is suffering from the following problems:
    1. Calibrating all four fisheye cameras together is unlikely to succeed due to the much more complex cost function
    2. fisheye camera models are hard to calibrate precisely with the original [Kalibr](https://github.com/ethz-asl/kalibr)
    3. Calibrating all cameras with only one rosbag introduces noisy pictures during calibration, making the solution unstable with significant reprojection errors.
    4. Once a calibration process fails, there are no middle results to resume the current calibration process, thereby causing much more effort, such as re-recording the calibration, re-running the calibration process, and waiting for a long time.

 ## Intro

Our method is based on both (Kalibr)[https://github.com/ethz-asl/kalibr] and (tartancalib)[https://github.com/castacks/tartancalib]. We use tartancalib to precisely calibrate the fisheye camera intrinsic parameters and extrinsic parameters of echo two of the nearby cameras.

![image-20240204173203608](https://raw.githubusercontent.com/Peize-Liu/my-images/master/202402041732792.png)

For omnidirectional depth estimation, every virtual stereo pair's parameters are only based on the extraction side fisheye camera pairs‘ parameters. For example, the front side virtual stereos parameters depend on CAM_D and CAM_A's intrinsic parameters, CAM_D and CAM_A's extrinsic parameters, and last, CAM_D's extrinsic parameters with IMU. 

So, we only need to calibrate every camera pair (CAM_A-CAM-B, CAM_B-CAM_C, CAM_C-CAM-D, CAM-D-CAM_A), there is no need to calibrate them all, and optimize all the parameters in one calibration.

Furthermore, splitting the calibration process reduces the waiting time and efforts to re-record all the calibration data.

### Prerequisites


### How to use
7 steps to get what we want.

1. record rosbag:
   1. image data
   2. imu data
2. extracting calibration rosbags
3. calibrating fisheye cameras' intrinsic parameters.
4. calibrating stereo pair cameras' extrinsic parameters.
5. calibrating fisheye cameras & imu extrinsic parameters.
6. generate virtual stereo input configurations(parameters from 3 4 5)
7. calibrating virtual stereos

refer to video











