import time

class PID:
    """
    A simple PID controller class.
    
    Attributes:
        Kp (float): Proportional gain.
        Kd (float): Derivative gain.
        Ki (float): Integral gain.
        prev_error (float): Previous error value.
        currtime (float): Current time.
        prevtime (float): Previous time.
        Cp (float): Proportional term.
        Ci (float): Integral term.
        Cd (float): Derivative term.
    """
    
    def __init__(self):
        """Initialize the PID controller with default gains and state."""
        self.Kp = 0
        self.Kd = 0
        self.Ki = 0
        self.Initialize()

    def SetKp(self, invar):
        """
        Set the proportional gain.
        
        Args:
            invar (float): Proportional gain value.
        """
        self.Kp = invar

    def SetKi(self, invar):
        """
        Set the integral gain.
        
        Args:
            invar (float): Integral gain value.
        """
        self.Ki = invar

    def SetKd(self, invar):
        """
        Set the derivative gain.
        
        Args:
            invar (float): Derivative gain value.
        """
        self.Kd = invar

    def SetPrevError(self, preverror):
        """
        Set the previous error value.
        
        Args:
            preverror (float): Previous error value.
        """
        self.prev_error = preverror

    def Initialize(self):
        """Initialize the PID controller state."""
        self.currtime = time.time()
        self.prevtime = self.currtime
        self.prev_error = 0
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

    def GenOut(self, error):
        """
        Compute the PID output value for a given error.
        
        Args:
            error (float): The current error value.
        
        Returns:
            float: The PID controller output.
        """
        self.currtime = time.time()
        dt = self.currtime - self.prevtime
        de = error - self.prev_error

        self.Cp = self.Kp * error
        self.Ci += error * dt

        self.Cd = 0
        if dt > 0:
            self.Cd = de / dt

        self.prevtime = self.currtime
        self.prev_error = error

        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)

# Example usage:
'''
pid = PID()
pid.SetKp(Kp)
pid.SetKd(Kd)
pid.SetKi(Ki)

fb = 0
outv = 0

PID_loop = True

while PID_loop:
    error = Sp - fb

    outv = pid.GenOut(error)
    AnalogOut(outv)

    time.sleep(0.05)

    fb = AnalogIn(fb_input)
    pass
'''
