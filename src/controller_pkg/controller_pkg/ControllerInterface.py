from controller_interfaces.msg import PIDGains
from typing import Optional
import numpy as np

class ControllerParametersInterface:

    def __init__(self):
        pass

class ControllerInterface:

    llim: float = -1
    ulim: float = 1

    def __init__(self):
        raise NotImplementedError
    
    def calculate_controls(self, input):
        raise NotImplementedError
    
    def set_parameters(self, params):
        raise NotImplementedError
    
    def get_parameters(self):
        raise NotImplementedError
    
    def saturate(self, u):
        return np.clip(u, self.llim, self.ulim)

    def to_dict(self):
        raise NotImplementedError
    
    def reset(self):
        raise NotImplementedError
    
    @staticmethod
    def from_dict(d: dict):
        raise NotImplementedError

class PIDControllerParameters(ControllerParametersInterface):

    kp: float
    ki: float
    kd: float
    Ts: float
    llim: float
    ulim: float
    sigma: float
    flag: bool

    def __init__(self, kp=float(1.0), ki=float(0.0), kd=float(0.0), Ts=float(0.01), \
                 llim=float(-100), ulim=float(100), sigma=float(0.01), flag=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.llim = llim
        self.ulim = ulim
        self.sigma = sigma
        self.flag = flag
        self.beta = (2.0*self.sigma-self.Ts)/(2.0*self.sigma+self.Ts)

class PIDController_Beard(ControllerInterface):

    params: PIDControllerParameters

    def __init__(self, params: PIDControllerParameters):
        self.kp = params.kp # Proportional control gain
        self.ki = params.ki # Integral control gain
        self.kd = params.kd # Derivative control gain
        self.llim = params.llim # The output saturates at this limit
        self.ulim = params.ulim
        self.sigma = params.sigma # dirty derivative bandwidth is 1/sigma
        self.beta = params.beta
        self.Ts = params.Ts # sample rate
        self.flag = params.flag

        # Initialize previous sample data vals
        self.y_dot = 0.0 # estimated derivative of y
        self.y_d1 = 0.0 # Signal y delayed by one sample
        self.error_dot = 0.0 # estimated derivative of error
        self.error_d1 = 0.0 # Error delayed by one sample
        self.integrator = 0.0 # integrator

    def calculate_controls(self, y_r, y):
        # Compute the current error
        error = y_r - y
        # integrate error using trapazoidal rule
        self.integrator = self.integrator \
        + (self.Ts/2) * (error + self.error_d1)
        # PID Control
        if self.flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot \
            + (1-self.beta)/self.Ts * (error - self.error_d1)
            # PID control
            u_unsat = self.kp*error \
            + self.ki*self.integrator \
            + self.kd*self.error_dot
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
            + (1-self.beta)/self.Ts * (y - self.y_d1)
            # PID control
            u_unsat = self.kp*error \
            + self.ki*self.integrator \
            - self.kd*self.y_dot
            # return saturated control signal
        u_sat = self.saturate(u_unsat)

        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator \
            + 1.0 / self.ki * (u_sat - u_unsat)
            # update delayed variables
            self.error_d1 = error
            self.y_d1 = y
        return u_sat

    def PD(self, y_r, y):
        # Compute the current error
        error = y_r - y
        # PD Control
        if self.flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot \
            + (1-self.beta)/self.Ts * (error - self.error_d1)
            # PD control
            u_unsat = self.kp*error \
            + self.kd*self.error_dot
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
            + (1-self.beta)/self.Ts * (y - self.y_d1)   

        # PD control
        u_unsat = self.kp*error \
        - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        # update delayed variables
        self.error_d1 = error
        self.y_d1 = y
        return u_sat

 

class PIDController_Beard_Vectorized(ControllerInterface):

    params: list[PIDControllerParameters]

    def __init__(self, params: list[PIDControllerParameters]):
        self.kp         = np.array([], dtype=np.float32) # Proportional control gain
        self.ki         = np.array([], dtype=np.float32) # Integral control gain
        self.kd         = np.array([], dtype=np.float32) # Derivative control gain
        self.llim       = np.array([], dtype=np.float32) # The output saturates at this limit
        self.ulim       = np.array([], dtype=np.float32)
        self.sigma      = np.array([], dtype=np.float32) # dirty derivative bandwidth is 1/sigma
        self.beta       = np.array([], dtype=np.float32)
        self.Ts         = np.array([], dtype=np.float32) # sample rate
        self.flag       = np.array([], dtype=np.float32)
        self.y_dot      = np.array([], dtype=np.float32)
        self.y_d1       = np.array([], dtype=np.float32)
        self.error_dot  = np.array([], dtype=np.float32)
        self.error_d1   = np.array([], dtype=np.float32)
        self.integrator = np.array([], dtype=np.float32)

        for p in params:
            self.kp = np.append(self.kp, p.kp)
            self.ki = np.append(self.ki, p.ki)
            self.kd = np.append(self.kd, p.kd)
            self.llim = np.append(self.llim, p.llim)
            self.ulim = np.append(self.ulim, p.ulim)
            self.sigma = np.append(self.sigma, p.sigma)
            self.beta = np.append(self.beta, p.beta)
            self.Ts = np.append(self.Ts, p.Ts)
            self.flag = np.append(self.flag, p.flag)

            # Initialize previous sample data vals
            self.y_dot = np.append(self.y_dot, 0.0)
            self.y_d1 = np.append(self.y_d1, 0.0)
            self.error_dot = np.append(self.error_dot, 0.0)
            self.error_d1 = np.append(self.error_d1, 0.0)
            self.integrator = np.append(self.integrator, 0.0)

    def calculate_controls(self, y_r: np.ndarray, y: np.ndarray):
        assert (type(y_r) == np.ndarray and type(y) == np.ndarray), "Make sure to input a list"
        assert (len(y_r) == len(y) == len(self.kp)),                "Input length mismatch to controller"
        # Compute the current error
        error = y_r - y
        # integrate error using trapazoidal rule
        self.integrator = self.integrator \
        + (self.Ts/2) * (error + self.error_d1)
        # PID Control
        if self.flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot \
            + (1-self.beta)/self.Ts * (error - self.error_d1)
            # PID control
            u_unsat = self.kp*error \
            + self.ki*self.integrator \
            + self.kd*self.error_dot
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
            + (1-self.beta)/self.Ts * (y - self.y_d1)
            # PID control
            u_unsat = self.kp*error \
            + self.ki*self.integrator \
            - self.kd*self.y_dot
            # return saturated control signal
        u_sat = self.saturate(u_unsat)

        # integrator anti - windup
        for i, ki in enumerate(self.ki):
            if ki != 0.0:
                self.integrator[i] = self.integrator[i] \
                + 1.0 / ki * (u_sat - u_unsat)
                # update delayed variables
                self.error_d1[i] = error[i]
                self.y_d1[i] = y[i]
        return u_sat



if __name__ == "__main__":
    l1 = PIDControllerParameters()
    l2 = PIDControllerParameters()
    p = PIDController_Beard_Vectorized([l1,l2])
    p.calculate_controls(np.array([10,100]),np.array([0,0]))