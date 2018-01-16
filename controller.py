"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 2
GRAVITY = -9.81


class NonlinearController(object):

    def __init__(
        self,
        Kp_pos=5.0,
        Kp_vel=4.0,
        Kp_alt=10.0,
        Kp_hdot=1.0,
        
        Kp_roll=6.5,#6.5,
        Kp_pitch=6.5,#6.5,
        Kp_yaw=6.5,
        
        Kp_p=20,#10,
        Kp_q=20,#10,
        Kp_r=20,
        
        max_tilt=0.5,
        max_ascent_rate=5,
        max_descent_rate=2,
        max_speed=5.0
    ):
        self.max_tilt = max_tilt
        self.max_ascent_rate = max_ascent_rate
        self.max_descent_rate = max_descent_rate
        self.Kp_hdot = Kp_hdot
        self.Kp_yaw = Kp_yaw
        self.Kp_r = Kp_r
        self.Kp_roll = Kp_roll
        self.Kp_p = Kp_p
        self.Kp_pitch = Kp_pitch
        self.Kp_q = Kp_q
        self.Kp_pos = Kp_pos
        self.Kp_vel = Kp_vel
        self.Kp_alt = Kp_alt
        self.max_speed = max_speed
    
    
    
    def position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0,0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        velocity_cmd = self.Kp_pos*(local_position_cmd-local_position)
        
        #Limit speed
        velocity_norm = np.sqrt(velocity_cmd[0]*velocity_cmd[0]+velocity_cmd[1]*velocity_cmd[1])
        
        if velocity_norm > self.max_speed:
            velocity_cmd = velocity_cmd*self.max_speed/velocity_norm
            
        acceleration_cmd = acceleration_ff + self.Kp_pos*(local_position_cmd-local_position) + self.Kp_vel*(local_velocity_cmd-local_velocity)
        #acceleration_cmd = acceleration_ff + self.Kp_vel*(velocity_cmd-local_velocity)
        return acceleration_cmd
    
    def altitude_control(self,altitude_cmd,vertical_velocity_cmd,altitude,vertical_velocity,attitude,acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: 2-element numpy array, desired vehicle 2D acceleration in the local frame [north, east]
        """
        
        hdot_cmd = self.Kp_alt*(altitude_cmd-altitude)+vertical_velocity_cmd
        
        #Limit the ascent/descent rate
        if(hdot_cmd > self.max_ascent_rate):
            hdot_cmd = self.max_ascent_rate
        elif(hdot_cmd < -self.max_descent_rate):
            hdot_cmd = -self.max_descent_rate
            
        acceleration_cmd = acceleration_ff + self.Kp_hdot*(hdot_cmd - vertical_velocity)
        
        R33 = np.cos(attitude[0])*np.cos(attitude[1])
        thrust = acceleration_cmd/R33
        return thrust
    
    def roll_pitch_controller(self,acceleration_cmd,attitude,thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll,pitch,yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        #Calculate rotation matrix        
        R = euler2RM(attitude[0],attitude[1],attitude[2])

        #Only command if positive thrust
        if thrust_cmd > 0.0:
            target_R13 = min(max(acceleration_cmd[0].item()/thrust_cmd.item(),-1.0),1.0)
            target_pitch = np.arcsin(-target_R13)
    
            target_R23 = min(max(acceleration_cmd[1].item()/thrust_cmd.item(),-1.0),1.0)
            target_roll = np.arctan2(target_R23,R[2,2])
    
            #Limit maximum tilt
            tilt_norm = target_roll*target_roll + target_pitch*target_pitch >self.max_tilt
            if abs(tilt_norm) >self.max_tilt:
                target_pitch = target_pitch*self.max_tilt/tilt_norm
                target_roll = target_roll*self.max_tilt/tilt_norm
                target_R13 = -np.sin(target_pitch)
                target_R23 = np.sin(target_roll)*np.cos(target_pitch)
            
            
            p_cmd = (1/R[2,2])*(R[1,0]*self.Kp_roll*(R[0,2]-target_R13)-R[0,0]*self.Kp_pitch*(R[1,2]-target_R23))
            q_cmd = (1/R[2,2])*(R[1,1]*self.Kp_roll*(R[0,2]-target_R13)-R[0,1]*self.Kp_pitch*(R[1,2]-target_R23))
        else: #Otherwise command no rate
            p_cmd = 0.0
            q_cmd = 0.0
            thrust_cmd = 0.0
        return np.array([p_cmd,q_cmd])
    
    def body_rate_control(self,body_rate_cmd,body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        Kp_rate = np.array([self.Kp_p,self.Kp_q,self.Kp_r])
        rate_error = body_rate_cmd-body_rate
        moment_cmd = np.multiply(Kp_rate,rate_error)
        
        return moment_cmd
    
    def yaw_control(self,yaw_cmd,yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        
        #Ensure the target is within range of 0 to 2*pi
        yaw_cmd = np.mod(yaw_cmd,2.0*np.pi)
        
        yaw_error = yaw_cmd-yaw
        if(yaw_error > np.pi):
            yaw_error = yaw_error-2.0*np.pi
        elif(yaw_error<-np.pi):
            yaw_error = yaw_error+2.0*np.pi
        
        yawrate_cmd = self.Kp_yaw*yaw_error
        return yawrate_cmd
    