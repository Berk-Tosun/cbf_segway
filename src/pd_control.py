import pybullet as p
import pybullet_data

import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import numpy as np
import cvxopt
import control

import time
import pathlib

#####################################################
# Define dynamics

""" 
The segway could be modeled as an inverted pendulum on cart.

Verified with:
https://ocw.mit.edu/courses/mechanical-engineering/2-003sc-engineering-dynamics-fall-2011/lagrange-equations/MIT2_003SCF11_rec8notes1.pdf

Full nonlinear dynamics are obtained with lagrangian.
2-dof; generalized coords, q = {x, theta}

1st eom: (q = x)
    x_ddot * (m1+m2) + 1/2 * m2 * l * theta_ddot * cos(theta) 
        - 1/2 * m2 * l * theta_dot**2 * sin(theta) = u

2nd eom: (q = theta)
    theta_ddot * (m2 * l**2 / 4 + I_G) + 1/2 * m2 * x_ddot * l * cos(theta)
        + 1/2 * m2 * g * l * sin(theta) = 0

define x = [x, x_dot, theta, theta_dot]
"""


# Writing in the simplest form, replace pendulum with simple pendulum
# and linearize; taken from Ogata's book (Modern Control - modelling chapter).
# m1: cart mass
# m2: mass at the end of pendulum
# l: length of pendulum

# ## Ogata
# def get_ss_A(m1, m2, l, g=-9.807):
#     return [
#         [0, 1, 0, 0],
#         [0, 0, m2 * g /m1, 0],
#         [0, 0, 0, 1],
#         [0, 0, -(m1 + m2) * g / (m1*l), 0]
#     ]

# def get_ss_B(m1, l):
#     return [0, 1/m1, 0, 1/(m1 * l)]

## Steve Brunton ~ adds damping on wheels
def get_ss_A(m1, m2, l, d=1, pendulum_up=True, g=-9.807):
    b = 1 if pendulum_up else -1
    return [
        [0, 1, 0, 0],
        [0, -d/m1, b*m2*g/m1, 0],
        [0, 0, 0, 1],
        [0, -b*d/(m1*l), -b*(m1+m2)*g/(m1*l), 0]
    ]

def get_ss_B(m1, l, pendulum_up=True):
    b = 1 if pendulum_up else -1
    return [0, 1/m1, 0, b/(m1 * l)]

m1 = 2
m2 = 1
l = 85.4 / 2 * 0.01  # m
wheel_dia = 0.39 # m

A = get_ss_A(m1, m2, l)
B = get_ss_B(m1, l)
C = [
    [1, 0, 0, 0],
    [0, 0, 1, 0]
]

model_plant = control.ss(A, B, C, 0)

Q = np.zeros((4, 4))
np.fill_diagonal(Q, [1, 1, 1, 1])
R = 100

K_lqr, _, _ = control.lqr(model_plant, Q, R)

"""
# It is possible to write the full non-linear form as well.
# (need to modify the equations by eliminating x_ddot and theta_ddot.)
# Available in steven brunton -> code:matlab -> chapter 8 (?)

def get_drift_term(m1, m2, l, I_G, g):
    def f(x): # x = [x, x_dot, theta, theta_dot]
        return [
            [0, 1, 0, 0],
            [0, 0, 1/2 * m2 * l * x[3]**2 * np.sin(x[2]), 1/2 * m2 * l * x_ddot * np.cos(x[2])] / (m1+m2),
            [0, 0, 0, 1],
            []
        ]
    return f

model_plant = control.NonlinearIOSystem(get_drift_term(), ...)
"""

#####################################################
# Set up simulation

dt = 1/240


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

p.loadURDF('plane.urdf')

robots = []

start_pos = [0, 0, 0.3]
start_orientation = p.getQuaternionFromEuler([0., 0, 0])
urdf_file = pathlib.Path("./URDF/cyberpod.urdf")
robot_0 = p.loadURDF(urdf_file.absolute().as_posix(), start_pos, 
    start_orientation)
robots.append(robot_0)

start_pos = [0, 2, 0.3]
start_orientation = p.getQuaternionFromEuler([0., 0, 0])
urdf_file = pathlib.Path("./URDF/cyberpod.urdf")
robot_1 = p.loadURDF(urdf_file.absolute().as_posix(), start_pos, 
    start_orientation)
robots.append(robot_1)

p.setGravity(0, 0, -9.807)
p.setTimeStep(dt)

joint_name2id = {}
for i in range(p.getNumJoints(robot_0)):
    joint_info = p.getJointInfo(robot_0, i)
    joint_name2id[joint_info[1].decode('UTF-8')] = joint_info[0]

# enable torque control

max_force = 0
for robot in robots:
    for joint_id in joint_name2id.values():
        p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL, 
                force=max_force)
torque_target = 0
torque_limit = 2
# torque_limit = 0.3

# A simple PD controller manually tuned by testing. 
# It aims to stabilize the pitch.
target_pitch = 0
k_p = 0.3
k_d = 0.1
# k_d = 1e-6

#####################################################
# Safety-critical control


def cbf(pitch):
    """ 
    h should be designed such that h > 0 corresponds to safe states.
    In this case limit the pitch angle to a range. 
    """
    allowable_deviation = np.pi / 12
    if pitch < 0:
        return -1 * pitch + allowable_deviation
    else:
       return pitch - allowable_deviation

def cbf_dot(pitch, pitch_dot):
    if pitch < 0:
        return -1 * pitch_dot
    else:
        return pitch_dot

# def get_A():
#     return

# def get_b():
#     return

def asif(nominal_control):
    """
    Active Set Invariance Filter implementation of cbf.
    Recall CBF formulation:

    Given function h, which is denoting safety (h(x) >= 0 is safe)
    Set invariance (or safety) can be achived by:

        Nagumo's theorem: (works on boundary of safe set)
            h_dot(x) >= 0

        CBF: (introduce controller and extend to whole set)
            h_dot(x, u) >= -gamma * h(x)

    Back to ASIF: minimally invasive control filtering; to do so
    it solves the following optimization problem:

        argmin || u - u_des ||**2
        s.t. A_ij <= b_ij forAll i =/= j

    Use qp to solve it. Reformulate the problem to match the standart form:

        min (1/2) * x.T @ P @ x + q.T @ x
        s.t. Gx <= h  
    """
    return nominal_control

    ## Following is more suitable for exponential cbf
    # P = cvxopt.matrix(np.eyes(2), tc='d')
    # q = cvxopt.matrix(-1 * nominal_control, tc='d')
    # G = cvxopt.matrix(get_A(), tc='d')
    # h = cvxopt.matrix(get_b(), tc='d')

    # cvxopt.solvers.options['show_progress'] = False
    # sol = cvxopt.solvers.qp(P, q, G, h)

    # return sol['x']

#####################################################
# Simulation

trqs_target = []
trqs_target_nom = []
trqs_applied = []

ptchs = []
ptchs_dot = []

prev_pitch = None
time_elapsed = 0.
# while p.isConnected():
while time_elapsed < 2:
    for robot in robots:
        position, orientation = p.getBasePositionAndOrientation(robot)
        x, y, z = position
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

        lin_vel_base, ang_vel_base = p.getBaseVelocity(robot)
        x_dot = lin_vel_base[0] * np.cos(yaw) + lin_vel_base[1] * np.sin(yaw) # (?)
        pitch_dot = ang_vel_base[1]

        ## lqr designed with linearized model
        force_cart = K_lqr @ [x, x_dot, pitch, pitch_dot]
        torque_target_nom = force_cart * wheel_dia / 2
        torque_target_nom = torque_target_nom / 2 # we have 2 identical wheels

        torque_target = asif(torque_target_nom)    
        torque_applied = np.clip(torque_target, -torque_limit, torque_limit)

        trqs_target.append(torque_target)
        trqs_target_nom.append(torque_target_nom)
        trqs_applied.append(torque_applied)
        
        ptchs.append(pitch)
        ptchs_dot.append(pitch_dot)

        p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=joint_name2id["w1_joint"],
                controlMode=p.TORQUE_CONTROL, 
                force=torque_applied)
        p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=joint_name2id["w2_joint"],
                controlMode=p.TORQUE_CONTROL, 
                force=torque_applied)

    p.stepSimulation()
    time_elapsed += dt
    time.sleep(dt)
    # if time_elapsed % 0.2 < 0.01:
    #     print(roll, pitch, yaw)     


#####################################################
# Results

trqs_target = np.array(trqs_target).reshape(-1, len(robots))
trqs_target_nom = np.array(trqs_target_nom).reshape(-1, len(robots)) 
trqs_applied = np.array(trqs_applied).reshape(-1, len(robots))

ptchs =  np.array(ptchs).reshape(-1, len(robots))
ptchs_dot = np.array(ptchs_dot).reshape(-1, len(robots))

for robot_no, _ in enumerate(robots):
    fig = plt.figure(f'{robot_no=}')

    t = np.linspace(0, time_elapsed, len(trqs_applied[:, robot_no])) 

    ax0 = fig.add_subplot(2, 1, 1)
    ax0.plot(t, trqs_applied[:, robot_no], alpha=0.7, label='applied')
    ax0.plot(t, trqs_target[:, robot_no], alpha=0.7, label='target:safe')
    ax0.plot(t, trqs_target_nom[:, robot_no], "--", alpha=0.7,
        label='target:nominal')
    ax0.set_title("Joint Torque")
    ax0.set_xlabel("time")
    ax0.legend()
    ax0.grid(True)

    ax1 = fig.add_subplot(2, 2, 3)
    ax1.plot(t, ptchs[:, robot_no], label="pitch")
    ax1.plot(t, ptchs_dot[:, robot_no], "--", label="pitch dot")
    ax1.set_title("States")
    ax1.set_xlabel("time")
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(2, 2, 4)
    ax2_plt = ax2.scatter(ptchs[:, robot_no], ptchs_dot[:, robot_no], c=t, alpha=0.2)
    ax2.set_title("States (Phase plane)")
    ax2.set_xlabel("Pitch")
    ax2.set_ylabel("Pitch dot")
    ax2.grid(True)
    ax2.axhline(color='black')
    ax2.axvline(color='black')
    divider = make_axes_locatable(ax2)
    cax = divider.append_axes('right', size='5%', pad=0.05)
    cbar = fig.colorbar(ax2_plt, cax=cax, orientation='vertical')
    cbar.set_label('Time')

    plt.tight_layout()
plt.show()

