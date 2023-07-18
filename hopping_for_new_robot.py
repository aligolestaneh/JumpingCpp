'''
Nooshin Kohli
Created: Jun. 12, 2023

This code makes robot jump for many times as wanted.
'''
import sys
import os
userdir = os.path.expanduser('~')
sys.path.append(userdir+"/projects/actuator")
from actuator import Actuator
from homing import home
import matplotlib.pyplot as plt
import time
import numpy as np
from extract_data import DATA
from robot_class import ROBOT
import RPi.GPIO as GPIO
from hardware import hardware
from control import Control
from cubic import cubic
# GPIO.setmode(GPIO.BOARD)
# contact_pin = 13
# GPIO.setup(contact_pin, GPIO.IN)



data = DATA()
hopping_data = data.get_pose_data_f()
hopping_velocity = data.get_vel_data_f()

with open("./just_stand/time_f40.txt", "r")as file:
    time_list= eval(file.readline())


def fixq3(q):
        r = 18./28.
        q[-1] = q[-1]*r
        return q
    
def fixq3inv(q):
        r = 28./18.
        q[-1] = q[-1]*r
        return q
################################################## ROBOT AND RBDL TRANSFORM
#################### convert position
def robot2rbdl(p1, p2, p3):
    q = [-(p1 - qh1), -(p2 - qh2), -(p3 - qh3)]
    q = fixq3(q)
    return q

def rbdl2robot(p1, p2, p3):
    p1, p2, p3 = fixq3inv([p1, p2, p3])
    rx = [-p1 + qh1, -p2 + qh2, -p3 + qh3]
    return rx

def detect_contact():
    a = GPIO.input(contact_pin)
    return a

def rbdl2robot_vel(v1, v2, v3):
    v1, v2, v3 = fixq3inv([v1, v2, v3])
    rx_dot = [-v1 + 0, -v2 + 0, -v3 + 0]
    return rx_dot
def robot2rbdl_vel(v1, v2, v3):
    qdot = [-(v1 - 0), -(v2 - 0), -(v3 - 0)]
    qdot = fixq3(qdot)
    return qdot

def cal_qdot(q_pre, q, t_pre, t_now):
    time_delta = t_now - t_pre
    qdot_cal = (q - q_pre)/time_delta
    return qdot_cal


def cubic_to_slip(t, t_td, t_des, q_td, qdot_td):
    T = t_des-t_td
    Tau = (t-t_td)/T
    if Tau>1:
        raise ValueError("tau > 1 in slip")
    q_td = np.array(q_td)
#     qdot_td = np.zeros(3)
    qdot_td = np.array(qdot_td)
    q_des = np.array(rbdl2robot(0.032,1.2014,-1.819)) # this hard code is from compression in slip model
    qdot_des = np.zeros(3)
    delta_q = q_des - q_td
    rho_0 = 0
    rho_1 = (T*qdot_td)/delta_q
    rho_2 = (-2*qdot_td-qdot_des)*T/delta_q + 3
    rho_3 = (qdot_td+qdot_des)*T/delta_q -2
    q = q_td + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q


    
def cubic_comp(t, t_lo, t_ap, hip2calf_len):
    '''
    This function gets tip of the foot close to hip.
    '''
    T = t_ap - t_lo
    Tau = (t-t_lo)/T
    if Tau>1:
        raise ValueError("tau > 1 in comp")
    y_ap = 0.01             ##the height we want to descend in flight mode 
    y_lo = 0
    ydot_lo = 0
    ydot_ap = 0
    delta_y = y_ap - y_lo
    rho_0 = 0
    rho_1 = (T*ydot_lo)/delta_y
    rho_2 = (-2*ydot_lo-ydot_ap)*T/delta_y + 3
    rho_3 = (ydot_ap+ydot_lo)*T/delta_y -2
    y = y_lo + delta_y*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    y = hip2calf_len + y
    
    return y

def cubic_decomp(t, t_td, t_ap, hip2calf_len):
    '''
    This function gets tip of the foot far from hip.
    '''
    T = t_td - t_ap
    Tau = (t-t_ap)/T
    if Tau>1:
        raise ValueError("tau > 1 in decomp")
    y_ap = 0                    
    y_td = -0.01                  ## the height we want to ascend in flight mode
    ydot_td = 0
    ydot_ap = 0
    delta_y = y_td - y_ap
    rho_0 = 0
    rho_1 = (T*ydot_ap)/delta_y
    rho_2 = (-2*ydot_ap-ydot_td)*T/delta_y + 3
    rho_3 = (ydot_ap+ydot_td)*T/delta_y -2
    y = y_ap + delta_y*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    y =  hip2calf_len + y  
    return y


def safety_check(pre_cond, cur_cond):
    if abs(cur_cond[0] - pre_cond[0]) > 0.15 :
        print("motor1 diff", cur_cond[0] - pre_cond[0])
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("big step detected!!!!!!!!!!")
        raise ValueError("motor1 had a big step!!!!!!!!!")
    if abs(cur_cond[1] - pre_cond[1]) > 0.15 :
        print("motor2 diff", cur_cond[1] - pre_cond[1])
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("big step detected!!!!!!!!!!")
        raise ValueError("motor2 had a big step!!!!!!!!!")
    if abs(cur_cond[2] - pre_cond[2]) > 0.2 :
        print("motor3 diff", cur_cond[2] - pre_cond[2])
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("big step detected!!!!!!!!!!")
        raise ValueError("motor3 had a big step!!!!!!!!!")
    
# plt.figure()
# plt.plot(time_list, hopping_data[0])#hip
# plt.plot(time_list, hopping_data[1])#thigh
# plt.plot(time_list, hopping_data[2])#calf
# plt.legend(["hip_q", "thigh_q", "calf_q"], loc="upper right")
# plt.title("desire q from RBDL")
# plt.figure()


robot_des_vel_hip = rbdl2robot_vel(hopping_velocity[0],0,0)
robot_des_vel_hip = robot_des_vel_hip[0]

robot_des_vel_thigh = rbdl2robot_vel(0,hopping_velocity[1],0)
robot_des_vel_thigh = robot_des_vel_thigh[1]

robot_des_vel_calf = rbdl2robot_vel(0,0,hopping_velocity[2])
robot_des_vel_calf = robot_des_vel_calf[2]


# plt.plot(time_list, robot_des_vel_hip)#hip
# plt.plot(time_list, robot_des_vel_thigh)#thigh
# plt.plot(time_list, robot_des_vel_calf)#calf
# plt.legend(["hip_vel", "thigh_vel", "calf_vel"], loc="upper right")
# plt.title("desire velocity for robot")
# plt.show()

hip_d = hopping_data[0]
thigh_d = hopping_data[1]
calf_d = hopping_data[2]

hip_vel_d = hopping_velocity[0]
thigh_vel_d = hopping_velocity[1]
calf_vel_d = hopping_velocity[2]




leg = Actuator('can0') #real robot
m1 = 0x06
m2 = 0x04
m3 = 0x05

# kp = 20
# kd = .5
# kp = 40
# kd = 0.8
kp = 60
kd = 0.9

leg.enable(m1)
leg.enable(m2)
leg.enable(m3)



q_home = home([leg, m1, m2, m3], kp=kp, kd=kd, enable_motors=False, disable_motors=False)
qh1 = q_home[0]
qh2 = q_home[1]
qh3 = q_home[2] - 3.9366 # different zero position

rx1 = leg.command(m1, qh1, 0, kp, kd, 0)
rx2 = leg.command(m2, qh2, 0, kp, kd, 0)
rx3 = leg.command(m3, 0, 0, 0, 0, 0) #TODO


############################################################## find the safe range during slip commands from data
hip_d_robot=[]
thigh_d_robot =[]
calf_d_robot = []

for i in range(len(hip_d)):
    hip_d_robot.append(rbdl2robot(hip_d[i],0,0)[0])
    thigh_d_robot.append(rbdl2robot(0,thigh_d[i],0)[1])
    calf_d_robot.append(rbdl2robot(0,0,calf_d[i])[2])
    
min_hip = min(hip_d_robot)
max_hip = max(hip_d_robot)
min_thigh = min(thigh_d_robot)
max_thigh= max(thigh_d_robot)
min_calf = min(calf_d_robot)
max_calf= max(calf_d_robot)
#############################################################
q_rbdl = robot2rbdl(rx1[1], rx2[1], rx3[1])
home_robot_q = [rx1[1],rx2[1],rx3[1]]

q_d_rbdl = [0.032, 1.201, -1.819]

############################################################## starting Initial positioning of HIP
xi = q_rbdl[0]
xf = q_d_rbdl[0]
dr = .02
if xi > xf: dr = -dr
pos = np.arange(xi, xf, dr)
dt = 1/len(pos)
tpre = time.time()
print("HIP:",pos)

time.sleep(2)
q_pre=rx1[1]

for p in pos:
    p_robot = rbdl2robot(p,0,0)
    diff = abs(q_pre - p_robot[0])
    if (diff > .1):
        print("diff:")
        print(diff)
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("Unsafe command to motors is detected!")
        time.sleep(0.5)
        raise ValueError("Unsafe command to motors is detected!")
    else:

        rx = leg.command(m1, p_robot[0], 0, kp, kd, 0)
        q_pre = rx[1]
        while(time.time() - tpre < dt): temp = 0
        tpre = time.time()
hip_motor_final = rx[1]
hip_rbdl_final = robot2rbdl(rx[1],0,0)[0]
time.sleep(1)

############################################################### starting initial positioning of calf

xi = q_rbdl[2]
xf = q_d_rbdl[2]
dr = .02
if xi > xf: dr = -dr
pos = np.arange(xi, xf, dr)
dt = 1/len(pos)
tpre = time.time()
print("CALF:", pos)
q_pre = rx3[1]
for p in pos:
    p_robot = rbdl2robot(0,0,p)
    diff = abs(q_pre - p_robot[2])

    if (diff > .12):
        print("diff:")
        print(diff)
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("Unsafe command to motors is detected!")
        time.sleep(0.5)
        raise ValueError("Unsafe command to motors is detected!")
    else:

        rx = leg.command(m3, p_robot[2], 0, kp, kd, 0)
        q_pre = rx[1]
        while(time.time() - tpre < dt): temp = 0
        tpre = time.time()

calf_motor_final = rx[1]
calf_rbdl_final = robot2rbdl(0,0,rx[1])[2]

time.sleep(1)
################################################################ starting initial positioning of THIGH
xi = q_rbdl[1]
xf = q_d_rbdl[1]
dr = .02
if xi > xf: dr = -dr
pos = np.arange(xi, xf, dr)
dt = 1/len(pos)
tpre = time.time()
q_pre = rx2[1]
print("THIGH:", pos)
for p in pos:
    p_robot = rbdl2robot(0,p,0)
    diff = abs(q_pre - p_robot[1])
    if (diff > .1):
        print("diff:")
        print(diff)
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("Unsafe command to motors is detected!")
        time.sleep(0.5)
        raise ValueError("Unsafe command to motors is detected!")
    else:

        rx = leg.command(m2, p_robot[1], 0, kp, kd, 0)
        q_pre = rx[1]
        while(time.time() - tpre < dt): temp = 0
        tpre = time.time()


thigh_motor_final = rx[1]
thigh_rbdl_final = robot2rbdl(0,rx[1],0)[1]

time.sleep(1)
final_pose_motor=[hip_motor_final,thigh_motor_final,calf_motor_final]
final_pose_rbdl = [hip_rbdl_final,thigh_rbdl_final,calf_rbdl_final]
print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
print("final position of motors: ", final_pose_motor)
print("final position in rbdl: ", final_pose_rbdl)



hardware = hardware(qh1,qh2,qh3)
counter = 1
t_first_cycle = time.time()
real_time_cycle = time.time() - t_first_cycle
n = 1000
real_time_cycle_list = [None]*n
data_cycle = [[None]]*n
contact_list_cycle = [None]*n
########################################### part1 lists
q_hip_com_1 = [None]*n
q_thigh_com_1 = [None]*n
q_calf_com_1 = [None]*n

q_hip_motor_1 = [None]*n
q_thigh_motor_1 = [None]*n
q_calf_motor_1 = [None]*n
real_time_list_1 = [None]*n
########################################### part3 lists
q_hip_com_3 = [None]*2000
q_thigh_com_3 = [None]*2000
q_calf_com_3 = [None]*2000

q_hip_motor_3 = [None]*2000
q_thigh_motor_3 = [None]*2000
q_calf_motor_3 = [None]*2000
real_time_list_3 = [None]*2000
control = Control(hardware,m1,m2,m3)
path =  "leg_RBDL.urdf"
robot = ROBOT(np.zeros((3, 1)), np.zeros((3, 1)), path) #model
data_counter = 0
hip_com,thigh_com,calf_com,hip_com_vel,thigh_com_vel,calf_com_vel = control.data_for_jump()
num = 0
time.sleep(2)
############################################################START HOPPING
while(counter<=1):
    kp = 90
    kd = 0.3

    time.sleep(0.1)
    dt = 0.01
    #dt = 0.0025

    first_check = 0
    t_first = time.time()
    
    for i in range(len(hip_d)):
        real_time = time.time()-t_first
        real_time_list_1[i] = (real_time)
        
        robot_hip = hip_com[i]
        
        robot_thigh = thigh_com[i]
        
        robot_calf = calf_com[i]
        ######################################################
        robot_hip_vel = hip_com_vel[i]
        
        robot_thigh_vel = thigh_com_vel[i]
        
        robot_calf_vel = calf_com_vel[i]
      
        
        if not ((min_hip-0.1) < robot_hip <(max_hip + 0.1)):
            print("hip:", robot_hip)
            rx1 = leg.command(m1, 0, 0, 0, 0, 0)
            rx2 = leg.command(m2, 0, 0, 0, 0, 0)
            rx3 = leg.command(m3, 0, 0, 0, 0, 0)
            leg.disable(m1)
            leg.disable(m2)
            leg.disable(m3)
            print ("Unsafe command to motors is detected!!!!!!!!!!")
            time.sleep(0.5)
            raise ValueError("your command position is not in the safe range!!!!!!!!!")
        elif not ((min_thigh-0.1) < robot_thigh <  (max_thigh+0.1)):
            print("thigh:", robot_thigh)
            rx1 = leg.command(m1, 0, 0, 0, 0, 0)
            rx2 = leg.command(m2, 0, 0, 0, 0, 0)
            rx3 = leg.command(m3, 0, 0, 0, 0, 0)
            leg.disable(m1)
            leg.disable(m2)
            leg.disable(m3)
            print ("Unsafe command to motors is detected!!!!!!!!!!")
            time.sleep(0.5)
            raise ValueError("your command position is not in the safe range!!!!!!!!!")

        elif not ((min_calf - 0.15) < robot_calf < (max_calf + 0.15)):
            print("calf:", robot_calf)
            rx1 = leg.command(m1, 0, 0, 0, 0, 0)
            rx2 = leg.command(m2, 0, 0, 0, 0, 0)
            rx3 = leg.command(m3, 0, 0, 0, 0, 0)
            leg.disable(m1)
            leg.disable(m2)
            leg.disable(m3)
            print ("Unsafe command to motors is detected!!!!!!!!!!")
            time.sleep(0.5)
            raise ValueError("your command position is not in the safe range!!!!!!!!!")

        else:
            
            rx1 = leg.command(m1, robot_hip, robot_hip_vel, kp, kd, 0)
            rx2 = leg.command(m2, robot_thigh, robot_thigh_vel, kp, kd, 0)
            rx3 = leg.command(m3, robot_calf, robot_calf_vel, kp, kd, 0)
            
            real_time_cycle = time.time() - t_first_cycle
            real_time_cycle_list[i] = real_time_cycle
            data_cycle[i] = [rx1,rx2,rx3]
#             contact_list_cycle.append(detect_contact())
            
            q_hip_com_1[i] = robot_hip
            q_thigh_com_1[i] = robot_thigh
            q_calf_com_1[i] = robot_calf
            
#             qdot_hip_com_1[i] = robot_hip_vel
#             qdot_thigh_com_1[i] = robot_thigh_vel
#             qdot_calf_com_1[i] = robot_calf_vel
            
            q_hip_motor_1[i] = rx1[1]
            q_thigh_motor_1[i] = rx2[1]
            q_calf_motor_1[i] = rx3[1]
            
#             qdot_hip_motor_1[i] = rx1[2]
#             qdot_thigh_motor_1[i] = rx2[2]
#             qdot_calf_motor_1[i] = rx3[2]
            num = num + 1 
            
            
            
#             contact_list.append(detect_contact())

            if ((time.time()-t_first) - real_time_list_1[num-1] > dt):
                print("loop is taking more time than expected!!!",(time.time()-t_first) - real_time_list_1[num-1])
            while((time.time()-t_first) - real_time_list_1[num-1] < dt):
                temp=0


  
            

#     #################################################################TOCHDOWN MOMENT
    time.sleep(0.18) ######################################## time in flight mode
    rx1 = leg.command(m1, rx1[1], 0, 30, 0.5, 0)
    rx2 = leg.command(m2, rx2[1], 0, 30, 0.5, 0)
    rx3 = leg.command(m3, rx3[1], 0, 30, 0.5, 0)
#     print("detect contact after while:", detect_contact())
#     print("real_time - t_td = ", real_time - t_td)

#     # print("qdot_robot in touchdown: ", [rx1[2],rx2[2],rx3[2]])
#     # print("qdot_rbdl in toucdown: ", robot2rbdl_vel(rx1[2],rx2[2],rx3[2]))

    t_first = time.time()
    real_time = time.time()- t_first 
    
    t_td = time.time()- t_first
    t_des = t_td + 0.3
            
    q_td = [rx1[1], rx2[1] , rx3[1]] ### q touch down given from last phase

    qdot_td = [0.0805,5.044, -5.0757]#hard code estimate
    qdot_td = np.array(rbdl2robot_vel(qdot_td[0],qdot_td[1],qdot_td[2]))
    i_3 = 0 ############################################################# counter for phase 3 loop
    q_hip_motor_3[i_3] = rx1[1]
    q_thigh_motor_3[i_3] = rx2[1]
    q_calf_motor_3[i_3] = rx3[1]
    dt = 0.0025
    kp=90
    kd=0.3
    q_des = np.array(rbdl2robot(0.032,1.2014,-1.819))
    qdot_des = np.array([0,0,0])
    td_to_slip = cubic(t_start=t_td, t_end=t_des, pos=q_td, vel=qdot_td, desired_pos=q_des, desired_vel=qdot_des)
    i += 1

    while(real_time <= t_des):
        real_time = time.time() - t_first
        real_time_list_3[i_3] = real_time
#         print(i_3)
       
        q_des = td_to_slip.answer(real_time)
#         print(q_des)
        
        rx1 = leg.command(m1, q_des[0], 0, kp, kd, 0)
        rx2 = leg.command(m2, q_des[1], 0, kp, kd, 0)
        rx3 = leg.command(m3, q_des[2], 0, kp, kd, 0)
#         
        real_time_cycle = time.time() - t_first_cycle
        real_time_cycle_list[i] = real_time_cycle
        data_cycle[i] = [rx1,rx2,rx3]
        i += 1 
        data_counter += 1
#         contact_list_cycle.append(detect_contact())
        q_hip_com_3[i_3] = q_des[0]
        q_thigh_com_3[i_3] = q_des[1]
        q_calf_com_3[i_3] = q_des[2]
        
   
        safety_check([q_hip_motor_3[i_3],q_thigh_motor_3[i_3],q_calf_motor_3[i_3]],[rx1[1],rx2[1],rx3[1]])    
# 
        q_hip_motor_3[i_3+1] = rx1[1]
        q_thigh_motor_3[i_3+1] = rx2[1]
        q_calf_motor_3[i_3+1] = rx3[1]
#         
#         real_time = time.time() - t_first
        i_3 += 1
        real_time = time.time() - t_first
        if ((time.time()-t_first) - real_time_list_3[i_3-1] > dt):
                print("loop is taking more time than expected!!!",(time.time()-t_first) - real_time_list_3[i_3-1])
    # print("this is the end of a jump")

#     kp = 60
#     kd = 0.9
#     q_slip = np.array(rbdl2robot(0.032,1.2014,-1.819))
#     rx1 = leg.command(m1, q_slip[0], 0, kp, kd, 0)
#     rx2 = leg.command(m2, q_slip[1], 0, kp, kd, 0)
#     rx3 = leg.command(m3, q_slip[2], 0, kp, kd, 0)
#    
#     real_time_cycle = time.time() - t_first_cycle
#     real_time_cycle_list[data_counter] = real_time_cycle
#     data_cycle[data_counter] = [rx1,rx2,rx3]
# #     contact_list_cycle.append(detect_contact())
# 
#     rx1 = leg.command(m1, q_slip[0], 0, kp, kd, 0)
#     rx2 = leg.command(m2, q_slip[1], 0, kp, kd, 0)
#     rx3 = leg.command(m3, q_slip[2], 0, kp, kd, 0)
#     if (abs(q_slip[0]-rx1[1]) > 0.15) or (abs(q_slip[1] - rx2[1])> 0.15) or (abs(q_slip[2] - rx3[1]) > 0.15) : 
#         raise ValueError("initial position is far from slip")
#     time.sleep(0.5)
    
    counter += 1
    data_counter += 1

######################################################## plotting begins
from plotting import Graphics

plt.figure()
plt.plot(q_hip_com_1,"--")
plt.plot(q_hip_motor_1)
plt.title("HIP")

plt.figure()
plt.plot(q_thigh_com_1,"--")
plt.plot(q_thigh_motor_1)
plt.title("THIGH")


plt.figure()
plt.plot(q_calf_com_1,"--")
plt.plot(q_calf_motor_1)
plt.title("CALF")
# plt.show()

plt.figure()
plt.plot(q_hip_com_3,"--")
plt.plot(q_hip_motor_3)
plt.title("HIP IN COMP")

plt.figure()
plt.plot(q_thigh_com_3,"--")
plt.plot(q_thigh_motor_3)
plt.title("THIGH IN COMP")

plt.figure()
plt.plot(q_calf_com_3,"--")
plt.plot(q_calf_motor_3, "--")
plt.title("CALF IN COMP")
plt.show()

# plots = Graphics()
# plots.phase_one(q_hip_com_1,q_thigh_com_1,q_calf_com_1,q_hip_motor_1,q_thigh_motor_1,q_calf_motor_1)
# 
# plt.show()





# q_hip_motor_3.pop(0)
# q_thigh_motor_3.pop(0)
# q_calf_motor_3.pop(0)
# 
# plt.figure()
# plt.plot(real_time_list_3, q_hip_com_3)
# plt.plot(real_time_list_3, q_thigh_com_3)
# plt.plot(real_time_list_3, q_calf_com_3)
# plt.plot(real_time_list_3, q_hip_motor_3)
# plt.plot(real_time_list_3, q_thigh_motor_3)
# plt.plot(real_time_list_3, q_calf_motor_3)
# plt.legend(["hip_com","thigh_com","calf_com","hip","thigh","calf"],loc="upper right")
# plt.show()
# 
# data_cycle = np.array(data_cycle)
# 
# plt.figure()
# plt.plot(real_time_cycle_list, data_cycle[:,0,1])   # [all datas, which motor, which data(id,q,qdot,curr)]
# plt.plot(real_time_cycle_list, data_cycle[:,1,1])
# plt.plot(real_time_cycle_list, data_cycle[:,2,1])
# plt.plot(real_time_cycle_list, contact_list_cycle)
# plt.title("whole cycle q from motor")
# plt.legend(["hip","thigh","calf","contact"], loc = "upper right")

# plt.figure()
# plt.plot(real_time_cycle_list, data_cycle[:,0,3])
# plt.plot(real_time_cycle_list, data_cycle[:,1,3])
# plt.plot(real_time_cycle_list, data_cycle[:,2,3])
# plt.plot(real_time_cycle_list, contact_list_cycle)
# plt.title("whole cycle torques from motor")
# plt.legend(["hip","thigh","calf","contact"], loc = "upper right")
# 
# plt.figure()
# plt.plot(real_time_list_2, x_tip_2)
# plt.plot(real_time_list_2,x_com_2,"--")
# 
# plt.plot(real_time_list_2, y_tip_2)
# plt.plot(real_time_list_2,y_com_2,"--")
# 
# plt.plot(real_time_list_2, z_tip_2)
# plt.plot(real_time_list_2,z_com_2,"--")
# 
# plt.title("position of the tip point in flight phase")
# plt.legend(["actual x position","x from qubic","actual y position","z from qubic","actual x position","z from qubic"],loc = "upper right")
# 
# 
# plt.show()










    

