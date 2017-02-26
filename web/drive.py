#!/usr/bin/env python

from flask import Flask, request, json, jsonify
import os
import subprocess
import rospy
from std_msgs.msg import Float32MultiArray

app = Flask(__name__, static_url_path='')

speed = 0.0
steering = 0.0

@app.route('/')
def root():
    return app.send_static_file('index.html')

@app.route('/steering', methods=['PUT'])
def set_steering():
    global steering
    json = request.get_json()
    steering = json['steering']

    update_arduino()
    return "Ok"

@app.route('/speed', methods=['PUT'])
def set_speed():
    global speed, steering
    json = request.get_json()
    speed = json['speed']

    if speed  == 0.0:
        steering = 0.0

    update_arduino()
    return "Ok"

@app.route('/sound', methods=['PUT'])
def sound():
    json = request.get_json()
    filename = json['filename']
    print "sound", filename

    subprocess.Popen(["omxplayer", "../../Sounds/" + filename]);
    return "Ok"

def update_arduino():
    rospy.logdebug("speed %f steering %f", speed, steering)

    msg = Float32MultiArray()
    msg.data.append(speed)
    msg.data.append(steering)
    motor_publisher.publish(msg)

@app.before_first_request
def init_ros():
    global node, motor_publisher

    node_name = "web%d" % os.getpid()
    node = rospy.init_node(node_name)

    motor_publisher = rospy.Publisher('/motors', Float32MultiArray, queue_size = 1)

    rospy.loginfo("Initialized ROS node " + node_name)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True)
