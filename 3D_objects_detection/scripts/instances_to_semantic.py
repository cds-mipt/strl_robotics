#!/usr/bin/python
import os
import rospy
import numpy as np
import pycocotools.mask as mask_utils
import cv2

from std_msgs.msg import Header
from sensor_msgs.msg import RegionOfInterest
from camera_objects_msgs.msg import ObjectArray, Object, RLE

rospy.init_node("to_semantic")

def mask_to_rle_msg_and_roi_msg(mask):
    mask_rle = mask_utils.encode(np.asfortranarray(mask))
    rle_msg = RLE(height=mask.shape[0], width=mask.shape[1], data=mask_rle['counts'])
    bbox = mask_utils.toBbox([mask_rle])[0]
    roi_msg = RegionOfInterest(x_offset=bbox[0], y_offset=bbox[1], width=bbox[2], height=bbox[3])
    return rle_msg, roi_msg

def rle_msg_to_mask(rle_msg):
    rle = {'counts': rle_msg.data,'size': [rle_msg.height, rle_msg.width]}
    mask = mask_utils.decode(rle)
    return mask

def remove_noize(img_gray):
    kernel = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(img_gray, kernel, iterations=2)
    dilated = cv2.dilate(eroded, kernel, iterations=2)
    return dilated

def set_value(mask_shape, bbox, coord, radius):
    mask = np.zeros(mask_shape, dtype=np.uint8)
    x, y = coord
    for i in range(bbox[0], bbox[1]):
        for j in range(bbox[2], bbox[3]):
            if (i - x)**2 + (j - y)**2 <= radius**2:
                mask[j][i] = 1
    return mask

def instances_to_semantic(objects_array):
    masks = [rle_msg_to_mask(object_.rle) for object_ in objects_array.objects]
    ojects_array_out = []
    if masks:
        masks = np.array(masks)
        semantic_mask = sum(masks)
        semantic_mask = np.clip(semantic_mask, 0, 1)
        semantic_mask = semantic_mask.astype(np.uint8)
        semantic_image = remove_noize(semantic_mask)
        _, cnts, _ = cv2.findContours(semantic_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_parameters = [cv2.minEnclosingCircle(c) for c in cnts]
        cnts_parameters = [param for param in cnts_parameters if param[1] > 5] #param == ((x, y), radius)

        if cnts_parameters:
            score = 1
            for cnt_parameter in cnts_parameters:
                (x, y), radius = cnt_parameter
                bbox = (max(0, int(x - radius)), 
                        min(int(x + radius), semantic_mask.shape[1]), 
                        max(0, int(y - radius)), 
                        min(int(y + radius), semantic_mask.shape[0]))
                mask = set_value(semantic_mask.shape, bbox, (int(x), int(y)), int(radius))
                rle_msg, roi_msg = mask_to_rle_msg_and_roi_msg(mask)
                object_ = Object(score=score, bbox=roi_msg, rle=rle_msg)
                ojects_array_out.append(object_)
                score += 1

    pub.publish(ObjectArray(header=Header(stamp=rospy.Time.now()), objects=ojects_array_out))


if __name__ == '__main__':    

    pub = rospy.Publisher('/realsense_back/color/semantic_objects', ObjectArray, queue_size=10)
    sub = rospy.Subscriber('/realsense_back/color/objects', ObjectArray, instances_to_semantic)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
