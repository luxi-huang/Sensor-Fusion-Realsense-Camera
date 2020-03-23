#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import yaml

def loadCalibrationFile(filename, cname):
    ci = CameraInfo()

    f = open(filename)
    calib = yaml.load(f)
    if calib is not None:
        ci.width = calib['image_width']
        ci.height = calib['image_height']
        ci.distortion_model = calib['distortion_model']
        ci.D = calib['distortion_coefficients']['data']
        ci.K = calib['camera_matrix']['data']
        ci.R = calib['rectification_matrix']['data']
        ci.P = calib['projection_matrix']['data']

    return ci

def callback(data):
    info.header = data.header
    pub.publish(info)

if __name__ == '__main__':
    rospy.init_node('camera_info_pub', anonymous=True)
    url = rospy.get_param('~url')
    rospy.Subscriber("image", Image, callback)
    info = loadCalibrationFile(url, '')
    print info
    pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)
    rospy.spin()
