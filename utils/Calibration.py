import rosbag
import cv2 as cv
import os
import yaml
from cv_bridge import CvBridge
import multiprocessing
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '.'))
import PrivateGlobalValue

def calibrate_mono(bag):
    import subprocess
    bagname = os.path.basename(bag)
    mono_topic = bagname.split(".")[0]
    bagpath = os.path.dirname(bag)
    cmd = f"""#!/bin/bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
export DISPLAY=:0.0
source /catkin_ws/devel/setup.bash &&
rosrun kalibr tartan_calibrate --bag /data/{bagname} --target /data/april_6x6.yaml --topics {mono_topic} --models omni-radtan --save_dir /data/{mono_topic} --dont-show-report """
    with open(f"{bagpath}/{mono_topic}.sh", "w") as f:
        f.write(cmd)
    os.system(f"chmod +x {bagpath}/{mono_topic}.sh")
    dockercmd = f"""docker run --rm -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" --entrypoint="/data/{mono_topic}.sh" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "{bagpath}:/data"  mortyl0834/omnitartancalib:quad_cam  """
    print(dockercmd)
    p_docker = subprocess.Popen(dockercmd, shell=True, stderr=subprocess.STDOUT)
    p_docker.wait()
    print("calibrate_mono done")

def calibrate_stereo(bag):
    import subprocess
    try:
        bagname = os.path.basename(bag)
        bagpath = os.path.dirname(bag)
        stereo_output_dir = bagname.split(".")[0]
        topic_former = stereo_output_dir.split("-")[0]
        topic_latter = stereo_output_dir.split("-")[1]
        print(f"topic_former {topic_former} topic_latter {topic_latter}")
        # remove former and latter
        if os.path.exists(f"{bagpath}/{stereo_output_dir}"):
            os.system(f"rm -r {bagpath}/{stereo_output_dir}")
        os.mkdir(f"{bagpath}/{stereo_output_dir}")
        intrinsic_file = open(f"{bagpath}/{stereo_output_dir}/intrinsic.yaml", "w")
        #generate better intrinsic
        former_cam_yaml = open(f"{bagpath}/{topic_former}/log1-camchain.yaml", "r")
        former_cam_intrinsic = yaml.load(former_cam_yaml, Loader=yaml.FullLoader)
        former_cam_yaml.close()
        former_cam_intrinsic = former_cam_intrinsic["cam0"]
        # print(f"former_cam_intrinsic {former_cam_intrinsic}")
        later_cam_yaml = open(f"{bagpath}/{topic_latter}/log1-camchain.yaml", "r")
        later_cam_intrinsic = yaml.load(later_cam_yaml, Loader=yaml.FullLoader)
        later_cam_yaml.close()
        later_cam_intrinsic = later_cam_intrinsic["cam0"]
        # print(f"later_cam_intrinsic {later_cam_intrinsic}")
        intrinsic_yaml = {}
        intrinsic_yaml["cam0"] = former_cam_intrinsic
        intrinsic_yaml["cam1"] = later_cam_intrinsic
        # print(f"intrinsic_yaml {intrinsic_yaml}")
        yaml.safe_dump(intrinsic_yaml, intrinsic_file,default_flow_style=False)
        intrinsic_file.close()
        cmd = f"""#!/bin/bash
    export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
    export DISPLAY=:0.0
    source /catkin_ws/devel/setup.bash &&
    rosrun kalibr tartan_calibrate --bag /data/{bagname} --target /data/april_6x6.yaml --topics {topic_former} {topic_latter} \
        --models omni-radtan  omni-radtan --save_dir /data/{stereo_output_dir} --dont-show-report \
        --intrinsic-prarameters /data/{stereo_output_dir}/intrinsic.yaml    """
        with open(f"{bagpath}/{stereo_output_dir}.sh", "w") as f:
            f.write(cmd)
        os.system(f"chmod +x {bagpath}/{stereo_output_dir}.sh")
        dockercmd = f"""docker run --rm -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" --entrypoint="/data/{stereo_output_dir}.sh" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v "{bagpath}:/data"  mortyl0834/omnitartancalib:quad_cam  """
        print(dockercmd)
        p_docker = subprocess.Popen(dockercmd, shell=True)
        p_docker.wait()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Stopping all tasks...")
        p_docker.terminate()
        p_docker.wait() 
    
def calibrate_imu(bag):
    import subprocess
    bagname = os.path.basename(bag)
    mono_topic = bagname.split(".")[0]
    bagpath = os.path.dirname(bag)
    cmd = f"""#!/bin/bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
export DISPLAY=:0.0
source /catkin_ws/devel/setup.bash &&
rosrun kalibr kalibr_calibrate_imu_camera  --bag /data/{bagname} --target /data/april_6x6.yaml --cam /data/{mono_topic}/log1-camchain.yaml \
    --imu /data/imu.yaml --dont-show-report """
    with open(f"{bagpath}/{mono_topic}_imu.sh", "w") as f:
        f.write(cmd)
    os.system(f"chmod +x {bagpath}/{mono_topic}_imu.sh")
    dockercmd = f"""docker run --rm -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" --entrypoint="/data/{mono_topic}_imu.sh" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "{bagpath}:/data" mortyl0834/omnitartancalib:kalibr  """
    print(dockercmd)
    p_docker = subprocess.Popen(dockercmd, shell=True, stderr=subprocess.STDOUT)
    p_docker.wait()
    print("calibrate_mono_imu done")

def calibrate_monos(ouput_bag_path):
    # calibrate
    try:
        pool = multiprocessing.Pool(processes=4)
        for i in range(1, 5):
            bag_path = ouput_bag_path + PrivateGlobalValue.rosbag_name[i] + ".bag"
            topics = PrivateGlobalValue.rosbag_name[i].split("-")[0]
            print("calibrate bag {}".format(topics))
            pool.apply_async(calibrate_mono, args=(bag_path,), error_callback=print)
        pool.close()
        pool.join()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Stopping all tasks...")
        pool.terminate()
        pool.join()
         
def calibrate_stereos(ouput_bag_path):
    # calibrate stereo
    try:
        pool = multiprocessing.Pool(processes=4)
        for i in range(5, 9):
            bag_path = ouput_bag_path + PrivateGlobalValue.rosbag_name[i] + ".bag"
            pool.apply_async(calibrate_stereo,args= (bag_path,),error_callback=print)
        pool.close()
        pool.join()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Stopping all tasks...")
        pool.terminate()
        pool.join()
    
def calibrate_cam_imu(ouput_bag_path):
    # calibrate imu
    try:
        pool = multiprocessing.Pool(processes=4)
        for i in range(1, 5):
            bag_path = ouput_bag_path + PrivateGlobalValue.rosbag_name[i] + ".bag"
            topics = PrivateGlobalValue.rosbag_name[i].split("-")[0]
            print("calibrate bag {}".format(topics))
            pool.apply_async(calibrate_imu, args=(bag_path,), error_callback=print)
        pool.close()
        pool.join()
    except KeyboardInterrupt:
        print("Ctrl+C detected. Stopping all tasks...")
        pool.terminate()
        pool.join()

def check_calibration(ouput_bag_path, calibration_type = "mono"):
    if calibration_type == "mono":
        bag_names = PrivateGlobalValue.rosbag_name[1:5]
    elif calibration_type == "stereo":
        bag_names = PrivateGlobalValue.rosbag_name[5:9]
    calibration_status = [False, False, False, False]
    iter = 0
    failed = False
    for bag_name in bag_names:
        bag_path = ouput_bag_path + "/" +  bag_name + "/log1-camchain.yaml"
        if os.path.exists(bag_path):
            calibration_status[iter] = True
        iter += 1
    for i in range(4):
        if calibration_status[i] == False:
            print("calibration failed for bag {} \
                  record new bag and extract under the same name".format(bag_names[i]))
            failed = True
    if failed==False:
        print("calibration success for all bags")
            
def check_imu_calibration(ouput_bag_path):
    topics = PrivateGlobalValue.rosbag_name[1:5]
    calibration_status = [False, False, False, False]
    iter = 0
    failed = False
    for topic in topics:
        bag_path = ouput_bag_path + "/" +  topic + "-camchain-imucam.yaml"
        if os.path.exists(bag_path):
            calibration_status[iter] = True
        else:
            print(f"imu calibration failed for {topic}")
            failed = True
        iter += 1
    if failed==False:
        print("imu calibration success for all bags")
    