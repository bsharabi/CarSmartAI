import time
import threading
import cv2

try:
    from greenlet import getcurrent as get_ident
except ImportError:
    try:
        from thread import get_ident
    except ImportError:
        from _thread import get_ident

class CameraEvent(object):
    """
    An Event-like class that signals all active clients when a new frame is available.
    """
    
    def __init__(self):
        """Initializes the CameraEvent instance with an empty events dictionary."""
        self.events = {}

    def wait(self):
        """
        Invoked from each client's thread to wait for the next frame.
        
        Returns:
            bool: True if the event was set, False otherwise.
        """
        ident = get_ident()
        if ident not in self.events:
            # This is a new client, add an entry for it in the self.events dict
            # Each entry has two elements, a threading.Event() and a timestamp
            self.events[ident] = [threading.Event(), time.time()]
        return self.events[ident][0].wait()

    def set(self):
        """
        Invoked by the camera thread when a new frame is available.
        Signals all clients and removes inactive ones.
        """
        now = time.time()
        remove = None
        for ident, event in self.events.items():
            if not event[0].isSet():
                # If this client's event is not set, then set it
                # Also update the last set timestamp to now
                event[0].set()
                event[1] = now
            else:
                # If the client's event is already set, it means the client
                # did not process a previous frame
                # If the event stays set for more than 5 seconds, then assume
                # the client is gone and remove it
                if now - event[1] > 5:
                    remove = ident
        if remove:
            del self.events[remove]

    def clear(self):
        """Invoked from each client's thread after a frame was processed."""
        self.events[get_ident()][0].clear()

class BaseCamera(object):
    """
    A base class for handling camera input in a separate background thread.
    
    Attributes:
        thread (threading.Thread): Background thread that reads frames from the camera.
        frame (bytes): Current frame stored by the background thread.
        last_access (float): Time of the last client access to the camera.
        event (CameraEvent): An instance of CameraEvent to signal frame availability.
    """
    
    thread = None  # background thread that reads frames from camera
    frame = None  # current frame is stored here by background thread
    last_access = 0  # time of last client access to the camera
    event = CameraEvent()

    def __init__(self):
        """Start the background camera thread if it isn't running yet."""
        if BaseCamera.thread is None:
            BaseCamera.last_access = time.time()

            # Start background frame thread
            BaseCamera.thread = threading.Thread(target=self._thread)
            BaseCamera.thread.start()

            # Wait until frames are available
            while self.get_frame() is None:
                time.sleep(0)

    def get_frame(self):
        """
        Return the current camera frame.
        
        Returns:
            bytes: The current frame.
        """
        BaseCamera.last_access = time.time()

        # Wait for a signal from the camera thread
        BaseCamera.event.wait()
        BaseCamera.event.clear()

        return BaseCamera.frame

    @staticmethod
    def frames():
        """
        Generator that returns frames from the camera.
        
        This method must be implemented by subclasses.
        
        Yields:
            bytes: The next frame.
            
        Raises:
            RuntimeError: If not implemented by subclasses.
        """
        raise RuntimeError('Must be implemented by subclasses.')

    @classmethod
    def _thread(cls):
        """
        Camera background thread.
        Reads frames from the camera and signals clients.
        """
        print('Starting camera thread.')
        frames_iterator = cls.frames()
        for frame in frames_iterator:
            BaseCamera.frame = frame
            BaseCamera.event.set()  # send signal to clients
            time.sleep(0)

        BaseCamera.thread = None
