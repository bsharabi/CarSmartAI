from importlib import import_module
import os
from flask import Flask, render_template, Response, send_from_directory
from flask_cors import CORS
from robot.camera_opencv import Camera
import threading

# Initialize the Flask app
app = Flask(__name__)
CORS(app, supports_credentials=True)
camera = Camera()

def gen(camera):
    """
    Video streaming generator function.
    
    Args:
        camera (Camera): An instance of the Camera class.
        
    Yields:
        bytes: The video frame in bytes format.
    """
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """
    Video streaming route. Put this in the src attribute of an img tag.
    
    Returns:
        Response: The video feed response.
    """
    return Response(gen(camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Set the directory path
dir_path = os.path.dirname(os.path.realpath(__file__))

@app.route('/api/img/<path:filename>')
def sendimg(filename):
    """
    Serve images from the img directory.
    
    Args:
        filename (str): The filename of the image.
        
    Returns:
        Response: The image file response.
    """
    return send_from_directory(dir_path + '/dist/img', filename)

@app.route('/js/<path:filename>')
def sendjs(filename):
    """
    Serve JavaScript files from the js directory.
    
    Args:
        filename (str): The filename of the JavaScript file.
        
    Returns:
        Response: The JavaScript file response.
    """
    return send_from_directory(dir_path + '/dist/js', filename)

@app.route('/css/<path:filename>')
def sendcss(filename):
    """
    Serve CSS files from the css directory.
    
    Args:
        filename (str): The filename of the CSS file.
        
    Returns:
        Response: The CSS file response.
    """
    return send_from_directory(dir_path + '/dist/css', filename)

@app.route('/api/img/icon/<path:filename>')
def sendicon(filename):
    """
    Serve icon images from the icon directory.
    
    Args:
        filename (str): The filename of the icon image.
        
    Returns:
        Response: The icon image file response.
    """
    return send_from_directory(dir_path + '/dist/img/icon', filename)

@app.route('/fonts/<path:filename>')
def sendfonts(filename):
    """
    Serve font files from the fonts directory.
    
    Args:
        filename (str): The filename of the font file.
        
    Returns:
        Response: The font file response.
    """
    return send_from_directory(dir_path + '/dist/fonts', filename)

@app.route('/<path:filename>')
def sendgen(filename):
    """
    Serve general files from the dist directory.
    
    Args:
        filename (str): The filename of the file.
        
    Returns:
        Response: The file response.
    """
    return send_from_directory(dir_path + '/dist', filename)

@app.route('/')
def index():
    """
    Serve the index.html file.
    
    Returns:
        Response: The index.html file response.
    """
    return send_from_directory(dir_path + '/dist', 'index.html')

class webapp:
    """
    Web application class for handling camera and server operations.
    """
    
    def __init__(self):
        """Initialize the webapp with a camera instance."""
        self.camera = camera

    def modeselect(self, modeInput):
        """
        Select the mode for the camera.
        
        Args:
            modeInput (str): The mode to select.
        """
        Camera.modeSelect = modeInput

    def colorFindSet(self, H, S, V):
        """
        Set the color finding parameters for the camera.
        
        Args:
            H (int): Hue value.
            S (int): Saturation value.
            V (int): Value (brightness) value.
        """
        camera.colorFindSet(H, S, V)

    def thread(self):
        """
        Run the Flask app in a separate thread.
        """
        app.run(host='0.0.0.0', threaded=True)

    def startthread(self):
        """
        Start the Flask app thread.
        """
        fps_threading = threading.Thread(target=self.thread)  # Define a thread for FPV and OpenCV
        fps_threading.setDaemon(False)  # 'True' means it is a front thread, it would close when the mainloop() closes
        fps_threading.start()  # Thread starts
