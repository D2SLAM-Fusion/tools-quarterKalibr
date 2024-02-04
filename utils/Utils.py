import cv2  as cv
import rosbag
import os
import yaml

def splitImage(img, num_subimages = 4):
    #Split image vertically
    h, w = img.shape[:2]
    sub_w = w // num_subimages
    sub_imgs = []
    for i in range(num_subimages):
        sub_imgs.append(img[:, i*sub_w:(i+1)*sub_w])
    return sub_imgs

def check_rosbag(image_topic, imu_topic, bag_path):
  if os.path.exists(bag_path) == False:
    print("Bag file:{} does not exist".format(bag_path))
    return False
  bag = rosbag.Bag(bag_path)
  if (bag.get_message_count(image_topic) <= 0):
    print('No message in topic: {}  check your configuration'.format(image_topic))
    bag.close()
    return False
  if (bag.get_message_count(imu_topic) <= 0):
    print('No message in topic: {}  check your configuration'.format(imu_topic))
    bag.close()
    return False
  bag.close()
  return True

def generate_d2vins_cinfiguration(ouput_bag_path):
    camera_a_yaml = open(ouput_bag_path + "/CAM_A/log1-camchain.yaml", "r")
    camera_a_intrinsic = yaml.load(camera_a_yaml, Loader=yaml.FullLoader)["cam0"]
    camera_a_yaml.close()
    camera_b_yaml = open(ouput_bag_path + "/CAM_B/log1-camchain.yaml", "r")
    camera_b_intrinsic = yaml.load(camera_b_yaml, Loader=yaml.FullLoader)["cam0"]
    camera_b_yaml.close()
    camera_c_yaml = open(ouput_bag_path + "/CAM_C/log1-camchain.yaml", "r")
    camera_c_intrinsic = yaml.load(camera_c_yaml, Loader=yaml.FullLoader)["cam0"]
    camera_c_yaml.close()
    camera_d_yaml = open(ouput_bag_path + "/CAM_D/log1-camchain.yaml", "r")
    camera_d_intrinsic = yaml.load(camera_d_yaml, Loader=yaml.FullLoader)["cam0"]
    camera_d_yaml.close()
    # generate d2vins configuration
    d2vins_config_yaml = open(ouput_bag_path + "fisheye_cams.yaml", "w")
    d2vins_config = {}
    d2vins_config["cam0"] = camera_a_intrinsic
    d2vins_config["cam1"] = camera_b_intrinsic
    d2vins_config["cam2"] = camera_c_intrinsic
    d2vins_config["cam3"] = camera_d_intrinsic
    cam_1_2_yaml = open(ouput_bag_path + "/CAM_A-CAM_B/log1-camchain.yaml", "r")
    cam_1_2_extrinsic = yaml.load(cam_1_2_yaml, Loader=yaml.FullLoader)["cam1"]["T_cn_cnm1"]
    cam_1_2_yaml.close()
    cam_2_3_yaml = open(ouput_bag_path + "/CAM_B-CAM_C/log1-camchain.yaml", "r")
    cam_2_3_extrinsic = yaml.load(cam_2_3_yaml, Loader=yaml.FullLoader)["cam1"]["T_cn_cnm1"]
    cam_2_3_yaml.close()
    cam_3_4_yaml = open(ouput_bag_path + "/CAM_C-CAM_D/log1-camchain.yaml", "r")
    cam_3_4_extrinsic = yaml.load(cam_3_4_yaml, Loader=yaml.FullLoader)["cam1"]["T_cn_cnm1"]
    cam_3_4_yaml.close()
    d2vins_config["cam1"]["T_cn_cnm1"] = cam_1_2_extrinsic
    d2vins_config["cam2"]["T_cn_cnm1"] = cam_2_3_extrinsic
    d2vins_config["cam3"]["T_cn_cnm1"] = cam_3_4_extrinsic
    # cam_imu
    cam_a_imu_yaml = open(ouput_bag_path + "/CAM_A-camchain-imucam.yaml", "r")
    cam_a_imu_extrinsic = yaml.load(cam_a_imu_yaml, Loader=yaml.FullLoader)["cam0"]["T_cam_imu"]
    cam_a_imu_yaml.close()
    cam_b_imu_yaml = open(ouput_bag_path + "/CAM_B-camchain-imucam.yaml", "r")
    cam_b_imu_extrinsic = yaml.load(cam_b_imu_yaml, Loader=yaml.FullLoader)["cam0"]["T_cam_imu"]
    cam_b_imu_yaml.close()
    cam_c_imu_yaml = open(ouput_bag_path + "/CAM_C-camchain-imucam.yaml", "r")
    cam_c_imu_extrinsic = yaml.load(cam_c_imu_yaml, Loader=yaml.FullLoader)["cam0"]["T_cam_imu"]
    cam_c_imu_yaml.close()
    cam_d_imu_yaml = open(ouput_bag_path + "/CAM_D-camchain-imucam.yaml", "r")
    cam_d_imu_extrinsic = yaml.load(cam_d_imu_yaml, Loader=yaml.FullLoader)["cam0"]["T_cam_imu"]
    cam_d_imu_yaml.close()
    d2vins_config["cam0"]["T_cam_imu"] = cam_a_imu_extrinsic
    d2vins_config["cam1"]["T_cam_imu"] = cam_b_imu_extrinsic
    d2vins_config["cam2"]["T_cam_imu"] = cam_c_imu_extrinsic
    d2vins_config["cam3"]["T_cam_imu"] = cam_d_imu_extrinsic
    yaml.safe_dump(d2vins_config, d2vins_config_yaml,default_flow_style=False)
    print("d2vins_config_yaml write into {}".format(ouput_bag_path + "/fisheye_cams.yaml"))
    d2vins_config_yaml.close()
                   