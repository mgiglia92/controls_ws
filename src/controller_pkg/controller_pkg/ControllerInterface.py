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

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, Ts=0.01, llim=-1, ulim=1, sigma=0.01, flag=True):
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

    # Saturate the controller output
    def saturate(self, u):
        return np.clip(u, self.llim, self.ulim)