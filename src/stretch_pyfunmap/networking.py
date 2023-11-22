import multiprocessing
from flask import Flask, render_template
from flask_socketio import SocketIO
import stretch_body.hello_utils as hu


def _setup_flask_socketio():
    app = Flask(__name__, template_folder='../../viz/static')
    socketio = SocketIO(app)
    # import logging
    # flask_logger = logging.getLogger('werkzeug')
    # flask_logger.setLevel(logging.ERROR)

    @app.route('/')
    def index():
        return render_template('index.html')

    @socketio.on('connect')
    def connect():
        pass
        # socketio.start_background_task(realsense_stream)

    @socketio.on('Slider value changed')
    def slider_change(message):
        message['who'] = 'slider2'
        socketio.emit('update value', message)

    try:
        socketio.run(app, host="0.0.0.0", allow_unsafe_werkzeug=True)
    except hu.ThreadServiceExit:
        pass


def start_networking():
    net_process = multiprocessing.Process(target=_setup_flask_socketio)
    net_process.start()
    return net_process
