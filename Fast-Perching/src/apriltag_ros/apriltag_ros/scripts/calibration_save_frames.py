#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import time
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage

class CompressedImageSaver(object):
    def __init__(self):
        # 参数
        self.save_dir = rospy.get_param("~save_dir", os.path.expanduser("~/Pictures/ros_captures"))
        self.save_rate = float(rospy.get_param("~save_rate", 1.0))  # Hz

        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo("Created directory: %s", self.save_dir)

        # 订阅 compressed 图像
        topic = "/camera/color/image/compressed"
        self.sub = rospy.Subscriber(topic, CompressedImage, self.img_cb, queue_size=1)
        rospy.loginfo("Subscribed to: %s", topic)

        # 保存控制
        self.lock = threading.Lock()
        self.latest_msg = None
        self.last_saved_time = 0.0

        # 用 Timer 控制 1Hz 保存（与图像到达频率解耦）
        if self.save_rate <= 0:
            rospy.logwarn("~save_rate <= 0; defaulting to 1.0 Hz")
            self.save_rate = 1.0
        period = 1.0 / self.save_rate
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_cb)

    def img_cb(self, msg):
        # 只缓存最新一帧
        with self.lock:
            self.latest_msg = msg

    def timer_cb(self, _event):
        # 每个周期保存一次“最新一帧”（如果有）
        with self.lock:
            msg = self.latest_msg
            self.latest_msg = None  # 取走后清空，避免重复保存同一帧

        if msg is None:
            # 当前周期没有新帧到达
            return

        try:
            # 解码 CompressedImage（jpeg/png）
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                rospy.logwarn("cv2.imdecode returned None; skip this frame.")
                return

            # 根据 format 决定扩展名
            fmt = (msg.format or "").lower()
            if "png" in fmt:
                ext = ".png"
            else:
                # 大多数是 jpeg；没识别也存 jpg
                ext = ".jpg"

            # 用 ROS 时间戳命名（更稳定）
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs
            fname = "color_{:010d}_{:09d}{}".format(sec, nsec, ext)
            fpath = os.path.join(self.save_dir, fname)

            # 写文件
            ok = cv2.imwrite(fpath, img)
            if ok:
                rospy.loginfo_throttle(2.0, "Saved: %s", fpath)
            else:
                rospy.logwarn("cv2.imwrite failed: %s", fpath)

        except Exception as e:
            rospy.logerr("Exception while saving image: %s", str(e))

def main():
    rospy.init_node("compressed_image_saver", anonymous=True)
    saver = CompressedImageSaver()
    rospy.loginfo("compressed_image_saver started. Saving at ~%.2f Hz to %s",
                  saver.save_rate, saver.save_dir)
    rospy.spin()

if __name__ == "__main__":
    main()