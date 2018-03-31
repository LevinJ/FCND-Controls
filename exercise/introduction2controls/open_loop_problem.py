import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from simplified_monorotor import Monorotor
import plotting
import testing
import trajectories

pylab.rcParams['figure.figsize'] = 10,10


class OpenLoopController:
    
    def __init__(self, vehicle_mass, initial_state=np.array([0,0])): 
        self.vehicle_mass  = vehicle_mass 
        
        # vehicle_state is the controller's BELIEF about the state
        # of the vehicle it is controlling. It doesn't know the 
        # TRUE state.
        self.vehicle_state = initial_state 
        self.g = 9.81
    
    def thrust_control(self, target_z, dt):
        """
        Returns a thrust which will be commanded to 
        the vehicle. This thrust should cause the vehicle
        to be at target_z in dt seconds.
        
        The controller's internal model of the vehicle_state
        is also updated in this method.
        """
        # 1. find target velocity needed to get to target_z
        current_z, current_z_dot = self.vehicle_state
        delta_z = target_z - current_z
        target_z_dot = delta_z / dt
        
        # 2. find target acceleration needed
        delta_z_dot = target_z_dot - current_z_dot
        target_z_dot_dot = delta_z_dot / dt
        
        # 3. find target NET force
        target_f_net = target_z_dot_dot * self.vehicle_mass
        
        # 4. find target thrust. Recall this equation:
        #    F_net = mg - thrust
        thrust = self.vehicle_mass * self.g - target_f_net
        
        # 5. update controller's internal belief of state
        self.vehicle_state += np.array([delta_z, delta_z_dot])
        
        return thrust 
    

total_time = 5.0
t = np.linspace(0.0,total_time,1000)
dt = t[1] - t[0]
z_path= 0.5*np.cos(2*t)-0.5

# plt.figure(figsize=(5,5))
# plt.ylabel("z (meters)")
# plt.xlabel("time (seconds)")
# plt.gca().invert_yaxis()
# plt.plot(t,z_path)
# plt.show()


MASS_ERROR = 1.02

drone = Monorotor()
drone_start_state = drone.X
drone_mass = drone.m

# The mass that the controller believes is not necessarily the
# true mass of the drone. This reflects the real world more accurately.
perceived_mass = drone_mass * MASS_ERROR
controller = OpenLoopController(perceived_mass, drone_start_state)

# 2. Run the simulation
drone_state_history = []
for target_z in z_path:
    drone_state_history.append(drone.X)
    thrust = controller.thrust_control(target_z, dt)
    drone.thrust = thrust
    drone.advance_state(dt)

# 3. Generate plots
z_actual = [h[0] for h in drone_state_history] 
plotting.compare_planned_to_actual(z_actual, z_path, t)










