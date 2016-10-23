from flask import Flask, request, json, jsonify
from werkzeug.contrib.fixers import ProxyFix
import serial
import subprocess

app = Flask(__name__, static_url_path='')

@app.route('/')
def root():
    return app.send_static_file('index.html')

@app.route('/steering', methods=['PUT'])
def steering():
    json = request.get_json()
    steering = json['steering']

    print "steering", steering
    sendArduion("steering=" + str(steering))
    return "Ok"

@app.route('/speed', methods=['PUT'])
def speed():
    json = request.get_json()
    speed = json['speed']

    print "speed", speed
    sendArduion("speed=" + str(speed))
    return "Ok"

@app.route('/sound', methods=['PUT'])
def sound():
    json = request.get_json()
    filename = json['filename']
    print "sound", filename

    subprocess.Popen(["omxplayer", "../../Sounds/" + filename]);
    return "Ok"

arduino = serial.Serial('/dev/ttyACM0', 115200)
def sendArduion(command):
    arduino.write(command + '\n')

# For gunicorn
app.wsgi_app = ProxyFix(app.wsgi_app)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True)
