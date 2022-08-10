#!/usr/bin/env python3

from time import time

import yaml
import numpy as np
import pandas as pd
import cv2
import faiss
import torch
import torchvision.transforms as transforms
import rospy
import message_filters
from PIL import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose, PoseStamped

from src.utils import convert_dict_to_tuple
from src.models.model_factory import get_model


class VisualPRNode:

    def __init__(self):
        rospy.init_node('visual_pr_node', log_level=rospy.DEBUG)

        config_path = rospy.get_param('~config_path')

        with open(config_path) as f:
            data = yaml.safe_load(f)

        self.config = convert_dict_to_tuple(data)

        self.encoder = self._init_model(self.config)
        self.img_transform = self._img_transform(resize=self.config.database.images.resize)

        self.db_index = self._load_index(self.config)

        self.db_csv = pd.read_csv(self.config.database.csv, index_col=0)

        self.sub_image = message_filters.Subscriber('image', CompressedImage, queue_size=1)
        self.pub_pose = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)

        self.sub_image.registerCallback(self.on_image)
    
        rospy.loginfo("visual_pr_node is ready")

    def on_image(self, compressed_image_msg: CompressedImage):
        rospy.logdebug("Received CompressedImage message")
        start_time = time()
        np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = self.img_transform(Image.fromarray(image_np, mode="RGB")).unsqueeze(0)
        if self.config.enable_cuda and torch.cuda.is_available():
            image = image.to('cuda')
        
        feature = self.encoder(image).detach().cpu().numpy()
        _, i_pred = self.db_index.search(feature, 1)

        prediction = self.db_csv.iloc[i_pred[0]]
        rospy.logdebug(f"Prediction: {prediction['image_ts']}")

        position = [prediction['t_x'], prediction['t_y'], prediction['t_z']]
        orientation = [prediction['q_x'], prediction['q_y'], prediction['q_z'], prediction['q_w']]
        pose_stamped_msg = self._create_pose_stamped_msg(position, orientation,
                                                         stamp=compressed_image_msg.header.stamp,
                                                         frame_id=compressed_image_msg.header.frame_id)

        self.pub_pose.publish(pose_stamped_msg)

        time_taken = time() - start_time
        rospy.logdebug(f"Processed the message and published a reply. Time taken: {time_taken*1000:.3f} ms")

    def _create_pose_stamped_msg(self, position, orientation, stamp, frame_id):
        pose_msg = Pose()
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = stamp
        pose_stamped_msg.header.frame_id = frame_id
        pose_stamped_msg.pose = pose_msg

        return pose_stamped_msg

    def _img_transform(self, resize=(480, 640)):
        if resize[0] > 0 and resize[1] > 0:
            return transforms.Compose([
                transforms.Resize(resize),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                    std=[0.229, 0.224, 0.225]),
            ])
        else:
            return transforms.Compose([
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                    std=[0.229, 0.224, 0.225]),
            ])

    def _init_model(self, config):
        encoder = get_model(config.model)
        weights = torch.load(config.model.weights, map_location=torch.device('cpu'))
        encoder.load_state_dict(weights['state_dict'])

        if config.enable_cuda and torch.cuda.is_available():
            rospy.logdebug('Moving model to cuda...')
            encoder = encoder.to('cuda')
        else:
            rospy.logdebug('Working on cpu')

        rospy.logdebug(f"Initialized {config.model.name} model with weights: {config.model.weights}")
        return encoder

    def _load_index(self, config):
        rospy.logdebug('Loading faiss index...')
        db_index_cpu = faiss.read_index(config.database.faiss_index)
        if config.enable_cuda and torch.cuda.is_available():
            rospy.logdebug('Moving index to cuda...')
            res = faiss.StandardGpuResources()
            db_index_gpu = faiss.index_cpu_to_gpu(res, 0, db_index_cpu)
            return db_index_gpu
        return db_index_cpu


def main(args=None):
    visual_pr_node = VisualPRNode()
    rospy.spin()


if __name__ == '__main__':
    main()
