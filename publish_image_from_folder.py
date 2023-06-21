#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Wenshan Wang, Yaoyu Hu,  CMU
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of CMU nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import cv2
import numpy as np
import os
from os import listdir
from os.path import isfile
import time
import argparse

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class PubImgFolder(object):
    def __init__(self, args):


        oxford_10_29 = 'data/Oxford/10-29-undistort/stereo/left_undistort'
        kitti_img = 'data/KITTI_10/image_left'
        img_file_name = oxford_10_29

        base_folder = '10-29-undistort'
        if args.base_img_folder is not None and args.base_img_folder != '':
            base_folder = args.base_img_folder
            if args.dataset == 'oxford':
                img_file_name = os.path.join('data/Oxford/', base_folder, 'stereo/left_undistort')
            elif args.dataset == '4seasons':
                img_file_name = os.path.join('data/Oxford/', base_folder, 'undistorted_images/cam0')
            elif args.dataset == 'dso':
                img_file_name = os.path.join('data/Oxford/', base_folder, 'left_undistort')


        kitti_pose = 'data/KITTI_10/pose_left.txt'
        pose_file_name = ''

        image_dir = rospy.get_param('~img_dir', args.base_img_folder)
        pose_file = rospy.get_param('~pose_file', pose_file_name )

        self.cv_bridge = CvBridge()
        self.img_pub = rospy.Publisher("rgb_image", Image, queue_size=10)
        self.caminfo_pub = rospy.Publisher("cam_info", CameraInfo, queue_size=10)
        self.scale_pub = rospy.Publisher("vo_scale", Float32, queue_size=10)

        files = listdir(image_dir)
        self.rgbfiles = [(image_dir +'/'+ ff) for ff in files if (ff.endswith('.png') or ff.endswith('.jpg'))]
        self.rgbfiles.sort()
        self.image_dir = image_dir

        print('Find {} image files in {}'.format(len(self.rgbfiles), image_dir))
        self.imgind = 0
        # skip frames
        if (args.skipped_frames is not None and int(args.skipped_frames) > 0):
            self.imgind = int(args.skipped_frames)

        if isfile(pose_file):
            self.poselist = np.loadtxt(pose_file)
            if len(self.poselist) != len(self.rgbfiles):
                print('Posefile {} does not have the same length with the rgb images'.format(pose_file))
                self.poselist=None
        else:
            self.poselist=None

    def caminfo_publish(self):
        caminfo = CameraInfo()
        # image info for EuRoC
        # caminfo.width   = 752
        # caminfo.height  = 480
        # caminfo.K[0]    = 458.6539916992
        # caminfo.K[4]    = 457.2959899902
        # caminfo.K[2]    = 367.2149963379
        # caminfo.K[5]    = 248.3750000000

        # # image info for KITTI_10
        # caminfo.width   = 1226
        # caminfo.height  = 370
        # caminfo.K[0]    = 707.0912
        # caminfo.K[4]    = 707.0912
        # caminfo.K[2]    = 601.8873
        # caminfo.K[5]    = 183.1104

        # marcelprasetyo: image info for Oxford. 
        # 983.044006 983.044006 643.646973 493.378998
        caminfo.width   = 1280
        caminfo.height  = 960
        caminfo.K[0]    = 983.044006
        caminfo.K[4]    = 983.044006
        caminfo.K[2]    = 643.646973
        caminfo.K[5]    = 493.378998

        # marcelprasetyo info for 4seasons
        # 501.4757919305817 501.4757919305817 421.7953735163109 167.65799492501083
        # caminfo.width   = 800
        # caminfo.height  = 400
        # caminfo.K[0]    = 501.4757919305817
        # caminfo.K[4]    = 501.4757919305817
        # caminfo.K[2]    = 421.7953735163109
        # caminfo.K[5]    = 167.65799492501083

        # marcelprasetyo info for singapore
        # 544 1024
        # 592.83883311, 594.716250455, 505.396947059, 289.281666658
        # caminfo.width   = 1024
        # caminfo.height  = 544
        # caminfo.K[0]    = 592.83883311
        # caminfo.K[4]    = 594.716250455
        # caminfo.K[2]    = 505.396947059
        # caminfo.K[5]    = 289.281666658

        self.caminfo_pub.publish(caminfo)

    def img_publish(self):
        if self.imgind >= len(self.rgbfiles):
            return False
        # publish GT scale from the posefile
        if self.poselist is not None and self.imgind > 0:
            trans = self.poselist[self.imgind][:3] - self.poselist[self.imgind-1][:3]
            dist = np.linalg.norm(trans) 
            scale_msg = Float32()
            scale_msg.data = dist
            self.scale_pub.publish(scale_msg)
        # publish image
        img = cv2.imread(self.rgbfiles[self.imgind])
        if len(img.shape)==2:
            img = np.stack([img, img, img])
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        # marcelprasetyo : record filename
        img_msg.header.frame_id = os.path.basename(self.rgbfiles[self.imgind])
        # img_msg.header.frame_id = 'map'
        # if self.img_pub.
        self.img_pub.publish(img_msg)
        self.imgind += 1
        return True

if __name__ == '__main__':
    rospy.init_node("img_folder_pub", log_level=rospy.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument("--base_img_folder", help="base folder in Rain_Datasets for Oxford dataset")
    parser.add_argument("--skipped_frames", help="how many frames to be skipped")
    parser.add_argument("--rate", help="how many images published per second (Hertz)")
    parser.add_argument("--dataset", help="oxford, 4seasons, dso")
    args = parser.parse_args()
    rate_hertz = 10
    if args.rate is not None and int(args.rate) > 0:
        rate_hertz = int(args.rate)

    node = PubImgFolder(args)
    node.caminfo_publish()
    time.sleep(0.1)
    rate = rospy.Rate(rate_hertz)

    

    while not rospy.is_shutdown():
        node.caminfo_publish()
        ret = node.img_publish()
        rate.sleep()
        if not ret:
            break
