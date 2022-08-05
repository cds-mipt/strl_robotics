"""
Extract images and lidar data from a rosbag.
"""

import os
import argparse

import pandas as pd
import cv2
import numpy as np
import rosbag
from tqdm import tqdm
from sensor_msgs.point_cloud2 import read_points

            
def find_common_timestamps(cam_lines, lidar_lines, odom_lines, max_delay):
    max_delay = int(max_delay * 1e+9)  # convert to nsec
    n_skipped = 0
    cur_lidar_idx = 0
    cur_odom_idx = 0
    output_timestamps_cam, output_timestamps_lidar, output_timestamps_odom = [], [], []
    for cam_ts in cam_lines:
        min_cam_lidar_ts_dif = max_delay
        for lidar_idx, lidar_ts in enumerate(lidar_lines[cur_lidar_idx:], start=cur_lidar_idx):
            cur_cam_lidar_ts_dif = abs(cam_ts - lidar_ts)
            if cur_cam_lidar_ts_dif < min_cam_lidar_ts_dif:
                min_cam_lidar_ts_dif = cur_cam_lidar_ts_dif
                cur_lidar_idx = lidar_idx
                min_odom_ts_dif = max_delay
                for odom_idx, odom_ts in enumerate(odom_lines[cur_odom_idx:], start=cur_odom_idx):
                    cur_odom_ts_dif = max(abs(odom_ts - cam_ts), abs(odom_ts - lidar_ts))
                    if cur_odom_ts_dif < min_odom_ts_dif:
                        min_odom_ts_dif = cur_odom_ts_dif
                        cur_odom_idx = odom_idx

        if max(abs(cam_ts - lidar_lines[cur_lidar_idx]),
               abs(cam_ts - odom_lines[cur_odom_idx]),
               abs(lidar_lines[cur_lidar_idx] - odom_lines[cur_odom_idx])) < max_delay:
            output_timestamps_cam.append(cam_ts)
            output_timestamps_lidar.append(lidar_lines[cur_lidar_idx])
            output_timestamps_odom.append(odom_lines[cur_odom_idx])
        else:
            print('Skipping  camera timestamp:', cam_ts)
            n_skipped += 1
    print(f"Finished. Skipped {n_skipped} timestamps in total.")
    return output_timestamps_cam, output_timestamps_lidar, output_timestamps_odom


def filter_timestamps(cam_timestamps, lidar_timestamps, odom_timestamps, filter_time):
    filter_time *= 1e+9  # to nsec
    filtered_cam_ts, filtered_lidar_ts, filtered_odom_ts = \
        [cam_timestamps[0]], [lidar_timestamps[0]], [odom_timestamps[0]]
    for i in range(1, len(cam_timestamps)):
        if abs(cam_timestamps[i] - filtered_cam_ts[-1]) > filter_time:
            filtered_cam_ts.append(cam_timestamps[i])
            filtered_lidar_ts.append(lidar_timestamps[i])
            filtered_odom_ts.append(odom_timestamps[i])
    return filtered_cam_ts, filtered_lidar_ts, filtered_odom_ts


def main(args):
    print('INFO: used topic names...')
    print('....: image_topic:{0:>30s}'.format(args.image_topic))
    print('....: lidar_topic:{0:>30s}'.format(args.lidar_topic))
    print('....: odom_topic:{0:>30s}'.format(args.odom_topic))

    bag_path = os.path.abspath(args.bag_path)

    if args.outdir != '':
        outdir = os.path.abspath(args.outdir)
    else:
        outdir = os.path.join(os.path.dirname(bag_path),
                              os.path.basename(bag_path).split('.bag')[0])
    os.makedirs(outdir, exist_ok=True)

    img_dir = os.path.join(outdir, 'images')
    os.mkdir(img_dir)

    lidar_dir = os.path.join(outdir, 'lidar')
    os.mkdir(lidar_dir)

    odom_dir = os.path.join(outdir, 'odom')
    os.mkdir(odom_dir)
    
    print('INFO: extract data to: {0}'.format(outdir))
    print('Opening bag file for reading ...')
    with rosbag.Bag(bag_path, mode='r') as bag:
        # reading timestamps for camera and lidar
        timestamps_lines_cam, timestamps_lines_lidar, timestamps_lines_odom = [], [], []
        print('Reading timestamps for camera:')
        with tqdm(total=bag.get_message_count(topic_filters=[args.image_topic])) as pbar:
            for topic, _, ts in bag.read_messages(topics=[args.image_topic]):
                timestamps_lines_cam.append(ts.to_nsec())
                pbar.update()
        pbar.close()
        print('Reading timestamps for lidar:')
        with tqdm(total=bag.get_message_count(topic_filters=[args.lidar_topic])) as pbar:
            for topic, _, ts in bag.read_messages(topics=[args.lidar_topic]):
                timestamps_lines_lidar.append(ts.to_nsec())
                pbar.update()
        pbar.close()
        print('Reading timestamps for odometry:')
        with tqdm(total=bag.get_message_count(topic_filters=[args.odom_topic])) as pbar:
            for topic, _, ts in bag.read_messages(topics=[args.odom_topic]):
                timestamps_lines_odom.append(ts.to_nsec())
                pbar.update()
        pbar.close()
        
        # finding common timestamps
        common_ts_cam, common_ts_lidar, common_ts_odom = \
            find_common_timestamps(timestamps_lines_cam, timestamps_lines_lidar, timestamps_lines_odom, args.max_delay)
        assert len(common_ts_cam) == len(common_ts_lidar) and len(common_ts_lidar) == len(common_ts_odom)
        print('Found '+str(len(common_ts_cam))+' common timestamps')

        # filtering timestamps
        common_ts_cam, common_ts_lidar, common_ts_odom = \
            filter_timestamps(common_ts_cam, common_ts_lidar, common_ts_odom, args.filter_time)

        ts_np = np.zeros((len(common_ts_cam), 2), dtype=int)
        ts_np[:, 0] = common_ts_cam
        ts_np[:, 1] = common_ts_lidar
        ts_df = pd.DataFrame(data=ts_np, columns=['image_ts', 'lidar_ts'])

        print('Extracting data:')
        poses_np = np.zeros((len(common_ts_odom), 7), dtype=float)
        pose_i = 0
        with tqdm(total=3*len(common_ts_cam)) as pbar:
            # extract data
            for topic, msg, ts in bag.read_messages(topics=[args.image_topic, args.lidar_topic, args.odom_topic]):
                if topic == args.image_topic:
                    if ts.to_nsec() in common_ts_cam:
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        file_name = os.path.join(img_dir, str(ts.to_nsec())+'.png')
                        cv2.imwrite(file_name, image_np)
                elif topic == args.lidar_topic:
                    if ts.to_nsec() in common_ts_lidar:
                        points = np.array(list(read_points(msg)), dtype=np.float32)
                        file_name = os.path.join(lidar_dir, str(ts.to_nsec())+'.bin')
                        points[:, :4].tofile(file_name)  # x, y, z, intensity
                elif topic == args.odom_topic:
                    if ts.to_nsec() in common_ts_odom:
                        t_x, t_y, t_z = msg.pose.pose.position.x, \
                                        msg.pose.pose.position.y, \
                                        msg.pose.pose.position.z
                        q_x, q_y, q_z, q_w = msg.pose.pose.orientation.x, \
                                             msg.pose.pose.orientation.y, \
                                             msg.pose.pose.orientation.z, \
                                             msg.pose.pose.orientation.w
                        poses_np[pose_i, :] = [t_x, t_y, t_z, q_x, q_y, q_z, q_w]
                        pose_i += 1
                pbar.update()
            assert pose_i == len(common_ts_odom)

        poses_df = pd.DataFrame(data=poses_np, columns=['t_x', 't_y', 't_z', 'q_x', 'q_y', 'q_z', 'q_w'])

        output_df = pd.concat([ts_df, poses_df], axis='columns')
        print("Output dataframe:")
        print(output_df.head())

        output_df.to_csv(os.path.join(outdir, 'database.csv'))


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='Extract images, lidar points and odometry from a rosbag')
    argparser.add_argument('bag_path',
                           help='path to .bag file')
    argparser.add_argument('--image_topic', default='/zed_node/left/image_rect_color/compressed',
                           help='left camera image topic name (type: sensor_msgs/CompressedImage)')
    argparser.add_argument('--lidar_topic', default='/velodyne_points',
                           help='lidar topic name (type: sensor_msgs/PointCloud2)')
    argparser.add_argument('--odom_topic', default='/husky_velocity_controller/odom',
                           help='odom topic name (type: nav_msgs/Odometry)')
    argparser.add_argument('--filter_time', default=2.0,
                           help='min time between two frames')
    argparser.add_argument('--outdir', '-o', default='',
                           help='output dirname')
    argparser.add_argument('--max_delay', '-d', type=float, default=0.05,
                           help='maximum delay between two timestamps that can be considered as one time, by default it should be 1/(2*fps) (strictly < )')
    args = argparser.parse_args()

    main(args)
