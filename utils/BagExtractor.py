import rosbag
from cv_bridge import CvBridge
import cv2 as cv
import tqdm
import os
import apriltag
import sys
import utils.Utils as common_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '.'))
import PrivateGlobalValue

#if want to modify, need to both modify 

def write_april_tag_target(ouput_dir):
    # write aprilgrid.yaml
    with open(ouput_dir + "april_6x6.yaml", "w") as f:
        f.write("target_type: 'aprilgrid'\n")
        f.write("tagCols: 6\n")
        f.write("tagRows: 6\n")
        f.write("tagSize: 0.088\n")
        f.write("tagSpacing: 0.3\n")


def extract_calibration_bag(bag_path, output_path, image_topic):
    current_step = 1
    bags = []
    bag = rosbag.Bag(bag_path)
    print("bag message count {}".format(bag.get_message_count(image_topic)))
    print("output path {}".format(output_path))
    bridge = CvBridge()
    # read image and split, check if image contains april tag
    if not os.path.exists(output_path):
        os.mkdir(output_path)
    ouput_bag_path = output_path + "/" + PrivateGlobalValue.rosbag_name[current_step] + ".bag"
    bags.append(ouput_bag_path)
    output_bag = rosbag.Bag(ouput_bag_path, 'w')
    pbar = tqdm.tqdm(total=bag.get_message_count(image_topic), colour="green")
    depth_stereo_bag_path = output_path + "/stereo_depth_calibration.bag"
    depth_stereo_bag = rosbag.Bag(depth_stereo_bag_path, 'w')
    for topic, msg, t in bag.read_messages():
        if topic == image_topic:
            if msg._type == "sensor_msgs/Image":
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            images = common_utils.splitImage(img, 4)
            gray_image = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            gray_images = common_utils.splitImage(gray_image, 4)
            #detect apriltag
            detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
            status = 0
            for i in range(4):
                # image_name = "image_" + str(i)
                # cv.imshow(image_name, images[i])
                # cv.waitKey(1)
                results = detector.detect(gray_images[i])
                if len(results) > 0:
                    status |= 1 << i
            if  PrivateGlobalValue.step_dict.get(status) - current_step == 1:
                current_step = PrivateGlobalValue.step_dict.get(status)
                output_bag.close()
                output_bag_path = output_path + PrivateGlobalValue.rosbag_name[current_step] + ".bag"
                bags.append(output_bag_path)
                output_bag = rosbag.Bag(output_bag_path, 'w')
            else :
                for i in range(4): # we write all image
                    write_image = images[i]
                    # change image to rosbag
                    camera_msg = bridge.cv2_to_imgmsg(write_image, encoding="bgr8")
                    camera_msg.header = msg.header
                    output_bag.write(PrivateGlobalValue.camera_topics[i], camera_msg, t)
            #collect data 
            if current_step >=5:
                depth_stereo_bag.write(topic, msg, t)
                # print("image write time stamp {}".format(t.to_sec()))
            pbar.update(1)
        else :
            rosbag.Bag.write(output_bag, topic, msg, t)
    write_april_tag_target(output_path)
    output_bag.close()
    depth_stereo_bag.close()
    return bags
  
  
def jump_sample_bag(input_bag, step, topics ,output_bag_path, start_time = 0):
  sample = 0
  input_bag_hdl = rosbag.Bag(input_bag)
  bag_name = os.path.basename(input_bag)
  sampled_bag_name = "/" + bag_name.split(".")[0] + f"{step}_sampled.bag".format(step)
  os.mkdir(output_bag_path)
  out_put_bag_hdl = rosbag.Bag(output_bag_path + sampled_bag_name, 'w')
  all_messages_num = 0
  for topic in topics:
    all_messages_num += input_bag_hdl.get_message_count(topic)
  pbar = tqdm.tqdm(total=all_messages_num/step, colour="green")
  topic_count_dict = {}
  #init diction
  for topic in topics:
    topic_count_dict[topic] = 0
  #TODO: add process
  bridge = CvBridge()
  t_0 = None
  for topic, msg, t in input_bag_hdl.read_messages():
    if t_0 is None:
      t_0 = t
    if (t - t_0).to_sec() < start_time:
      continue
    if topic in topics:
      if topic_count_dict[topic] % step == 0:
        out_put_bag_hdl.write(topic, msg, t)
        # print("write topic: {} time: {}".format(topic, t))
        pbar.update(1)
      topic_count_dict[topic] += 1
    else:
      out_put_bag_hdl.write(topic, msg, t)
  out_put_bag_hdl.close()
  
  
  
def extract_individual_calibration_bag(bag_path, original_topics, extract_topics, steps):
  message_cout = 0
  camera_topics = ["CAM_A", "CAM_B", "CAM_C", "CAM_D"]
  camera_index_index = {"CAM_A": 0, "CAM_B": 1, "CAM_C": 2, "CAM_D": 3}
  bag = rosbag.Bag(bag_path)
  output_path = os.path.dirname(bag_path) + "/extract_output/"
  if not os.path.exists(output_path):
    os.mkdir(output_path)
  cam_name = extract_topics
  cam_names = cam_name.split(",")
  step = steps
  write_index = []
  for name in cam_names:
    write_index.append(camera_index_index[name])
  print("read camera topics from {}".format(cam_names))
  if len(cam_names) == 1:
    output_path += cam_names[0]
  elif len(cam_names) > 1:
    for name in cam_names:
      output_path += name + "-"
  output_path += ".bag"
  ouput_bag = rosbag.Bag(output_path, 'w')
  bridge = CvBridge()
  for topic, msg, t in bag.read_messages():
    if topic == original_topics:
      if msg._type == "sensor_msgs/Image":
          img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
      else:
          img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
      message_cout += 1
      if message_cout % step != 0:
        continue
      images = common_utils.splitImage(img, 4)
      for i in write_index:
        image_msg = bridge.cv2_to_imgmsg(images[i], encoding="passthrough")
        image_msg.header = msg.header
        ouput_bag.write(camera_topics[i], image_msg, t)
    else:
      ouput_bag.write(topic, msg, t)
  ouput_bag.close()
  bag.close()
  print("Extracted bag saved to: ", output_path)
  