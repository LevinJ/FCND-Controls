import numpy as np 
import math
from math import sin, cos, tan, sqrt
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from mpl_toolkits.mplot3d import Axes3D
# import jdc
import random

from solution import UDACITYDroneIn3D, UDACITYController
import testing

pylab.rcParams['figure.figsize'] = 10, 10


class DroneIn3D(UDACITYDroneIn3D):
    
    def __init__(self,
                k_f = 1.0,
                k_m = 1.0,
                m = 0.5,
                L = 0.566, # full rotor to rotor distance
                i_x = 0.1,
                i_y = 0.1,
                i_z = 0.2):
        
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.l = L / (2*sqrt(2)) # perpendicular distance to axes
        self.i_x = i_x
        self.i_y = i_y
        self.i_z = i_z
        
        # x, y, y, phi, theta, psi, 
        # x_dot, y_dot, z_dot, p, q, r
        self.X=np.array([0.0,0.0,0.0,0.0,0.0,0.0,
                         0.0,0.0,0.0,0.0,0.0,0.0])
        self.omega = np.array([0.0,0.0,0.0,0.0])
        
        self.g = 9.81
        return
    # euler angles [rad] (in world / lab frame)
    @property
    def phi(self):
        return self.X[3]
    
    @property
    def theta(self):
        return self.X[4]
    
    @property
    def psi(self):
        return self.X[5]
    
    # body rates [rad / s] (in body frame)
    @property 
    def p(self):
        return self.X[9]
    
    @property
    def q(self):
        return self.X[10]
    
    @property 
    def r(self):
        return self.X[11]
    # forces from the four propellers [N]
    @property
    def f_1(self):
        f = self.k_f*self.omega[0]**2
        return f
    
    @property 
    def f_2(self):
        f = self.k_f*self.omega[1]**2
        return f
    
    @property 
    def f_3(self):
        f = self.k_f*self.omega[2]**2
        return f
    
    @property 
    def f_4(self):
        f = self.k_f*self.omega[3]**2
        return f
    
    # collective force
    @property
    def f_total(self):
        f_t = self.f_1 + self.f_2 + self.f_3 + self.f_4
        return f_t
    # reactive moments [N * m]
    @property
    def tau_1(self):
        tau = self.k_m * self.omega[0]**2
        return tau
        
    @property
    def tau_2(self):
        tau = -self.k_m * self.omega[1]**2
        return tau
    
    @property
    def tau_3(self):
        tau = self.k_m * self.omega[2]**2
        return tau
    
    @property
    def tau_4(self):
        tau = -self.k_m * self.omega[3]**2
        return tau
    
    # moments about axes [N * m]
    @property
    def tau_x(self):
        tau = self.l*(self.f_1 + self.f_4 - self.f_2 - self.f_3)
        return tau
    
    @property
    def tau_y(self):
        tau = self.l*(self.f_1 + self.f_2 - self.f_3 - self.f_4)
        return tau
    
    @property
    def tau_z(self):
        tau = self.tau_1 + self.tau_2 + self.tau_3 + self.tau_4
        return tau
    def set_propeller_angular_velocities(self,
                                    c,
                                    u_bar_p,
                                    u_bar_q,
                                    u_bar_r):
    
    # TODO replace with your own implementation.
    #   note that this function doesn't return anything
    #   it just sets self.omega
    # 
    # self.omega[0] = 
    # self.omega[1] =
    # self.omega[2] = 
    # self.omega[3] = 
    
        c_bar = -c * self.m / self.k_f
        p_bar = u_bar_p * self.i_x / (self.k_f * self.l)
        q_bar = u_bar_q * self.i_y / (self.k_f * self.l)
        r_bar = u_bar_r * self.i_z / self.k_m 

        omega_4 = (c_bar + p_bar - r_bar - q_bar)/4
        omega_3 = (r_bar - p_bar)/2 + omega_4
        omega_2 = (c_bar - p_bar)/2 - omega_3
        omega_1 = c_bar - omega_2 - omega_3 - omega_4
        
        self.omega[0] = -np.sqrt(omega_1)
        self.omega[1] = np.sqrt(omega_2)
        self.omega[2] = -np.sqrt(omega_3)
        self.omega[3] = np.sqrt(omega_4)
    def R(self):
    
        # TODO replace with your own implementation 
        #   according to the math above
        # 
        # return rotation_matrix
        
        r_x = np.array([[1, 0, 0],
                    [0, cos(self.phi), -sin(self.phi)],
                    [0, sin(self.phi), cos(self.phi)]])
    
        r_y = np.array([[cos(self.theta), 0, sin(self.theta)],
                        [0, 1, 0],
                        [-sin(self.theta), 0, cos(self.theta)]])
        
        r_z = np.array([[cos(self.psi), -sin(self.psi), 0],
                        [sin(self.psi), cos(self.psi), 0],
                        [0,0,1]])
        
        r = np.matmul(r_z,np.matmul(r_y,r_x))
        return r
    def linear_acceleration(self):
    
        # TODO replace with your own implementation
        #   This function should return a length 3 np.array
        #   with a_x, a_y, and a_z
        
        R = self.R()
        g = np.array([0,0,self.g]).T
        c = -self.f_total
        accelerations =  g + np.matmul(R,np.array([0,0,c]).T) / self.m
        return accelerations
    def get_omega_dot(self):
    
        # TODO replace with your own implementation
        # return np.array([p_dot, q_dot, r_dot])
        
        p_dot = self.tau_x/self.i_x - self.r * self.q *(self.i_z - self.i_y)/self.i_x
        q_dot = self.tau_y/self.i_y - self.r * self.p *(self.i_x - self.i_z)/self.i_y
        r_dot = self.tau_z/self.i_z - self.q * self.p *(self.i_y - self.i_x)/self.i_z
        
        return np.array([p_dot,q_dot,r_dot])
    def get_euler_derivatives(self):
    
        # TODO - replace with your own implementation
        #   return np.array([phi_dot, theta_dot, psi_dot])
        euler_rot_mat= np.array([[1, sin(self.phi) * tan(self.theta), cos(self.phi) * tan(self.theta)],
                                 [0, cos(self.phi), -sin(self.phi)],
                                 [0, sin(self.phi) / cos(self.theta), cos(self.phi) / cos(self.theta)]])
        
        
        derivatives_in_bodyframe =np.array([self.p,self.q,self.r]).T
        
        euler_dot= np.matmul(euler_rot_mat,derivatives_in_bodyframe)
        
        return euler_dot
    def advance_state(self, dt):
    
        # TODO replace this with your own implementation
        # 
        #   make sure this function returns the new state! 
        #   Some of the code that calls this function will expect
        #   it to return the state, so simply updating self.X 
        #   is not enough (though you should do that in this
        #   method too.)
        
        euler_dot_lab = self.get_euler_derivatives()
        body_frame_angle_dot = self.get_omega_dot()
        accelerations = self.linear_acceleration()
        
        X_dot = np.array([self.X[6],
                          self.X[7],
                          self.X[8],
                          euler_dot_lab[0],
                          euler_dot_lab[1],
                          euler_dot_lab[2],
                          accelerations[0],
                          accelerations[1],
                          accelerations[2],
                          body_frame_angle_dot[0],
                          body_frame_angle_dot[1],
                          body_frame_angle_dot[2]])
        
        self.X = self.X + X_dot * dt
        
        return self.X
class Controller(UDACITYController):
    
    def __init__(self,
                z_k_p=1.0,
                z_k_d=1.0,
                x_k_p=1.0,
                x_k_d=1.0,
                y_k_p=1.0,
                y_k_d=1.0,
                k_p_roll=1.0,
                k_p_pitch=1.0,
                k_p_yaw=1.0,
                k_p_p=1.0,
                k_p_q=1.0,
                k_p_r=1.0):
        
        
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.x_k_p = x_k_p
        self.x_k_d = x_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r
        
        print('x: delta = %5.3f'%(x_k_d/2/math.sqrt(x_k_p)), ' omega_n = %5.3f'%(math.sqrt(x_k_p)))
        print('y: delta = %5.3f'%(y_k_d/2/math.sqrt(y_k_p)), ' omega_n = %5.3f'%(math.sqrt(y_k_p)))
        print('z: delta = %5.3f'%(z_k_d/2/math.sqrt(z_k_p)), ' omega_n = %5.3f'%(math.sqrt(z_k_p)))
        
        self.g= 9.81
    def lateral_controller(self,
                      x_target,
                      x_dot_target,
                      x_dot_dot_target,
                      x_actual,
                      x_dot_actual,
                      y_target,
                      y_dot_target,
                      y_dot_dot_target,
                      y_actual,
                      y_dot_actual,
                      c):
    
        # TODO replace with your own implementation
        # return b_x_c, b_y_c
        
        x_err = x_target - x_actual
        x_err_dot = x_dot_target - x_dot_actual

        p_term_x = self.x_k_p * x_err
        d_term_x = self.x_k_d * x_err_dot

        x_dot_dot_command = p_term_x + d_term_x + x_dot_dot_target
        
        b_x_c = x_dot_dot_command/c
        
        
        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual

        p_term_y = self.y_k_p * y_err
        d_term_y = self.y_k_d * y_err_dot

        y_dot_dot_command = p_term_y + d_term_y + y_dot_dot_target
        
        b_y_c = y_dot_dot_command/c
        
        return b_x_c, b_y_c
    def roll_pitch_controller(self,
                          b_x_c_target,
                          b_y_c_target,
                          rot_mat):
    
        # TODO replace with your own implementation
        # return p_c, q_c
        
        b_x = rot_mat[0,2]
        b_x_err = b_x_c - b_x
        b_x_p_term = self.k_p_roll * b_x_err
        
        b_y = rot_mat[1,2]
        b_y_err = b_y_c - b_y  
        b_y_p_term = self.k_p_pitch * b_y_err
        
        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term
        
        rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]
        
        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
        p_c = rot_rate[0]
        q_c = rot_rate[1]
        
        return p_c, q_c
    
# testing.test_exercise_1_1(DroneIn3D)
# testing.test_exercise_1_2(DroneIn3D)
# testing.test_exercise_1_3(DroneIn3D)
# testing.test_exercise_2_1(DroneIn3D)
# testing.test_exercise_3_1(DroneIn3D)
testing.test_exercise_3_2(DroneIn3D)
testing.test_exercise_4_1(Controller)
testing.test_exercise_4_2(Controller)
    
    