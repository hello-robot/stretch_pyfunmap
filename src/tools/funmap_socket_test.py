import io
import cv2
import sys
import numpy as np
import stretch_pyfunmap.robot
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

robot = stretch_pyfunmap.robot.FunmapRobot()
app = Flask(__name__)
socketio = SocketIO(app)

values = {
    'slider1': 0,
}

def realsense_stream():
    while True:
        frames = robot.head_cam.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        encode_success, buffer = cv2.imencode(".jpg", color_image)
        if not encode_success:
            print('FAIL: encode failed')
            sys.exit(1)
        io_buffer = io.BytesIO(buffer)
        socketio.emit('realsense stream',  {'data': io_buffer.read()})

@app.route('/')
def index():
    return render_template('index.html',**values)

@socketio.on('connect')
def test_connect():
    socketio.start_background_task(realsense_stream)

@socketio.on('Slider value changed')
def value_changed(message):
    values[message['who']] = message['data']
    message['who'] = 'slider2'
    emit('update value', message, broadcast=True)

if __name__ == '__main__':
    socketio.run(app, host="0.0.0.0")
    # https://stackoverflow.com/questions/53399948/flask-socketio-send-images
