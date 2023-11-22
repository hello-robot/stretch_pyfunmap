import time
import numpy as np
import threading
import multiprocessing
from multiprocessing import shared_memory
from flask import Flask, render_template
from flask_socketio import SocketIO
import stretch_body.hello_utils as hu


def _setup_flask_socketio(socketio_events, shared_mem_blueprint):
    app = Flask(__name__, template_folder='../../viz/static')
    socketio = SocketIO(app)
    # import logging
    # flask_logger = logging.getLogger('werkzeug')
    # flask_logger.setLevel(logging.ERROR)

    def socketio_sharedmem_reader():
        shared_mem = {}
        for shm_name, shm_dtype_str, shm_shape, _ in shared_mem_blueprint:
            shared_mem[shm_name] = (shm_shape, np.dtype(shm_dtype_str), shared_memory.SharedMemory(name=shm_name))
        while True:
            if socketio_events['color_image_reply'].is_set():
                shm_color_image = shared_mem['shmcolorimagenp']
                color_image = np.ndarray(shape=shm_color_image[0], dtype=shm_color_image[1], buffer=shm_color_image[2].buf)
                import io
                import cv2
                import sys
                encode_success, buffer = cv2.imencode(".jpg", color_image)
                if not encode_success:
                    print('socketio_sharedmem_reader FAIL: encode failed')
                    sys.exit(1)
                io_buffer = io.BytesIO(buffer)
                socketio.emit('realsense stream',  {'data': io_buffer.read()})
                socketio_events['color_image_reply'].clear()

    @app.route('/')
    def index():
        return render_template('index.html')

    @socketio.on('connect')
    def connect():
        socketio.start_background_task(socketio_sharedmem_reader)

    @socketio.on('Slider value changed')
    def slider_change(message):
        message['who'] = 'slider2'
        socketio.emit('update value', message)
        socketio_events['color_image_call'].set()

    try:
        socketio.run(app, host="0.0.0.0", allow_unsafe_werkzeug=True)
    except hu.ThreadServiceExit:
        pass

def _setup_sharedmem_service(robot, socketio_events, shared_mem_blueprint, shutdown_flag):
    shared_mem = {}
    for shm_name, shm_dtype_str, shm_shape, shm_size in shared_mem_blueprint:
        shared_mem[shm_name] = (shm_shape, np.dtype(shm_dtype_str), shared_memory.SharedMemory(create=True, size=shm_size, name=shm_name))
    while not shutdown_flag.is_set():
        if socketio_events['color_image_call'].is_set():
            shm_color_image = shared_mem['shmcolorimagenp']
            dst = np.ndarray(shape=shm_color_image[0], dtype=shm_color_image[1], buffer=shm_color_image[2].buf)
            frames = robot.head_cam.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            dst[:] = color_image[:]
            socketio_events['color_image_call'].clear()
            socketio_events['color_image_reply'].set()

def start_networking(shared_mem_blueprint, robot):
    # setup socketio event flags
    socketio_events = {
        'color_image_call': multiprocessing.Event(),
        'color_image_reply': multiprocessing.Event(),
    }

    # setup shared memory service thread
    shm_serve_thread_shutdown_flag = threading.Event()
    shm_serve_thread = threading.Thread(target=_setup_sharedmem_service, args=(robot, socketio_events, shared_mem_blueprint, shm_serve_thread_shutdown_flag,))
    shm_serve_thread.start()
    time.sleep(0.1)

    # setup flask socketio process
    flask_process = multiprocessing.Process(target=_setup_flask_socketio, args=(socketio_events, shared_mem_blueprint,))
    flask_process.start()

    return flask_process, shm_serve_thread, shm_serve_thread_shutdown_flag
