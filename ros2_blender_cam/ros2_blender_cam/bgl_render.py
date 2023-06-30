#!/usr/bin/env python

# Derived from https://github.com/ubi-agni/sr_common/blob/ubi-noetic-devel/sr_description/blender/scripts/actuate_render.py
# distributed under CC-BY-4.0 (https://creativecommons.org/licenses/by/4.0/)
# author Guillaume Walck (University of Bielefeld)
# modifications : port to ROS 2 and factorize with a base Camera class

# Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Author: Guillaume Walck
#

import bpy
import bgl
import sys
import numpy as np
from mathutils import Matrix
from datetime import datetime
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image as ROSImage, CameraInfo

class Camera():
    def __init__(self, node, render_mode, cam_prefix, encoding, channel, frame_id='camera_link', topic='image', blender_cam_name='Camera'):
        # init parameters
        self.node = node
        self.render_mode = render_mode
        
        self.name = blender_cam_name
        self.frame_id = frame_id
        self.topic = topic
        self.cam_prefix = cam_prefix
        self.encoding = encoding
        self.color_channel = channel
        
        # prepare internal variables
        # size of the camera image (fixed by the scene)
        self.WIDTH = bpy.data.scenes['Scene'].render.resolution_x
        self.HEIGHT = bpy.data.scenes['Scene'].render.resolution_y
        self.latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # actual width of the view port
        self.width = self.WIDTH
        self.height = self.HEIGHT
        
        self.u_0 = self.WIDTH / 2
        self.v_0 = self.HEIGHT / 2
        Tx = 0
        Ty = 0
        R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.image_focal = bpy.data.cameras[self.name].lens
        print("camera lens " + str(self.image_focal))
        alpha_u = self.WIDTH * self.image_focal / bpy.data.cameras[self.name].sensor_width

        self.clip_near = bpy.data.cameras[self.name].clip_start
        self.clip_far = bpy.data.cameras[self.name].clip_end
        
        print("clipping :" + str(self.clip_near) + ", " + str(self.clip_far))
        intrinsic = Matrix.Identity(3)
        
        intrinsic[0][0] = alpha_u
        intrinsic[1][1] = alpha_u
        intrinsic[0][2] = self.width / 2.0
        intrinsic[1][2] = self.height / 2.0

        # camera info
        # source code from Morse DepthCamera
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id 
        self.camera_info.height = self.height
        self.camera_info.width = self.width
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [0]
        self.camera_info.k = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                              intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                              intrinsic[2][0], intrinsic[2][1], intrinsic[2][2]]
        self.camera_info.r = R
        self.camera_info.p = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], Tx,
                              intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], Ty,
                              intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0]

        # internal buffers
        self.framebuffer = bgl.Buffer(bgl.GL_INT, 1)
        self.viewport_info = bgl.Buffer(bgl.GL_INT, 4)
        
        # depth image
        self.image_msg = ROSImage()
        self.image_msg.width = self.WIDTH
        self.image_msg.height = self.HEIGHT
        self.image_msg.encoding = self.encoding
        self.image_msg.step = self.width * self.color_channel
        self.image_msg.header.frame_id = self.frame_id
        self.pub_image = self.node.create_publisher(ROSImage, self.cam_prefix + self.topic, 10)

        self.pub_camera_info = self.node.create_publisher(CameraInfo, self.cam_prefix + "/camera_info", self.latching_qos)
        # publish the message once
        self.camera_info.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_camera_info.publish(self.camera_info)

    # compute correct zoom factor to fill view (probably for 50 mm lens)
    # https://blender.stackexchange.com/questions/16493/is-there-a-way-to-fit-the-viewport-to-the-current-field-of-view
    def fit_zoom(self, factor):
        return self.image_focal*(2*math.sqrt(factor) - math.sqrt(2))
    
    def draw(self, cam, context):
        timenow = self.node.get_clock().now()
        self.image_msg.header.stamp = timenow.to_msg()
        self.pub_image.publish(self.image_msg)
        self.camera_info.header.stamp = timenow.to_msg()
        self.pub_camera_info.publish(self.camera_info)

class DepthCamera(Camera):

    def __init__(self, node, render_mode):
        super().__init__(node=node, render_mode=render_mode, cam_prefix='depth', topic='/image_rect_raw', encoding="32FC1", channel=4)
        print("Constructed DepthCamera")
    
    # Draw function which copies data from the 3D View
    # https://blender.stackexchange.com/questions/190140/copy-framebuffer-of-3d-view-into-custom-frame-buffer
    def draw(self, cam, context):

        bgl.glEnable(bgl.GL_DEPTH)
        # get currently bound framebuffer
        bgl.glGetIntegerv(bgl.GL_DRAW_FRAMEBUFFER_BINDING, self.framebuffer)
        # get information on current viewport
        bgl.glGetIntegerv(bgl.GL_VIEWPORT, self.viewport_info)
        self.width = self.viewport_info[2]
        self.height = self.viewport_info[3]
        # HACK skip processing if this is the wrong VIEW_3D region
        # since larger than HEIGHT pixels
        if self.height > self.HEIGHT+100:
            return

        self.pixelBuffer = bgl.Buffer(bgl.GL_FLOAT, self.width * self.height)
        # obtain depthbuffer from the framebuffer
        bgl.glReadPixels(0, 0, self.WIDTH, self.HEIGHT, bgl.GL_DEPTH_COMPONENT, bgl.GL_FLOAT, self.pixelBuffer)
    
        # copy only desired size and denormalize
        # https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer/6657284#6657284
        zfar = self.clip_far
        znear = self.clip_near
        distanceToPlane = (-zfar * znear / (np.array(self.pixelBuffer[0:self.WIDTH*self.HEIGHT], dtype=np.float32, copy=False) * (zfar - znear) - zfar))
        # convert to pinhole model (distance to cam center not to camera plane)
        depthimage = distanceToPlane.reshape(self.HEIGHT,self.WIDTH)# * self.projectionLUT
        # output data to ROS
        self.image_msg.data = np.flipud(depthimage).tobytes()
        
        super().draw(cam, context)

class ColorCamera(Camera):

    def __init__(self, node, render_mode):
        if 'A' in render_mode:
          super().__init__(node=node, render_mode=render_mode, cam_prefix='color', topic='/image_rect', encoding="rgba8", channel=4)
        else:
          super().__init__(node=node, render_mode=render_mode, cam_prefix='color', topic='/image_rect', encoding="rgb8", channel=3)
        print("Constructed ColorCamera")

    def draw(self, cam, context):

        bgl.glEnable(bgl.GL_DEPTH)
        # get currently bound framebuffer
        bgl.glGetIntegerv(bgl.GL_DRAW_FRAMEBUFFER_BINDING, self.framebuffer)
        # get information on current viewport
        bgl.glGetIntegerv(bgl.GL_VIEWPORT, self.viewport_info)
        self.width = self.viewport_info[2]
        self.height = self.viewport_info[3]
        # HACK skip processing if this is the wrong VIEW_3D region
        # since larger than HEIGHT pixels
        if self.height > self.HEIGHT+100:
            return

        self.colorpixelBuffer = bgl.Buffer(bgl.GL_BYTE, self.width * self.height * self.color_channel)
        # obtain color buffer from the framebuffer
        if "A" in self.render_mode:
            bgl.glReadPixels(0, 0, self.WIDTH, self.HEIGHT, bgl.GL_RGBA, bgl.GL_UNSIGNED_BYTE, self.colorpixelBuffer)
        else:
            bgl.glReadPixels(0, 0, self.WIDTH, self.HEIGHT, bgl.GL_RGB, bgl.GL_UNSIGNED_BYTE, self.colorpixelBuffer)
        colorbuffer = np.array(self.colorpixelBuffer[0:self.WIDTH*self.HEIGHT*self.color_channel], dtype=np.uint8, copy=False)
        # output data to ROS
        self.image_msg.data = np.flip(colorbuffer.astype(np.uint8).reshape(self.HEIGHT,self.WIDTH,self.color_channel),axis=0).tobytes()

        super().draw(cam, context)
    
