import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import threading
from threading import Thread
import math
#Do not touch these lines..
##########################################
rospy.init_node('scan_values')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
##########################################
front=0
side=0
is_left=True
side_index=180 #front index does not change


#This is a very important function. It says, when the drone navigate, then wait for its arrival before doing nother task.
def wait_arrival(tolerance=0.2):
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

        
#This function is to adjust the drone to the side wall. 
def side_function():
    print('\nside_function started\n')
    data=rospy.wait_for_message('/lidar',LaserScan) #reading the laserscan data.. 
    if (globals()['is_left']):
        while data.ranges[180]<0.6 and data.ranges[180]>0.8: 
            if (data.ranges[180]<0.6):
                diff=0.7-data.ranges[180]
                navigate(x=0, y=-1*diff, z=0, frame_id='navigate_target')
                wait_arrival()
                data=rospy.wait_for_message('/lidar',LaserScan)
                continue
            if (data.ranges[180]>0.8):
                diff=data.ranges[180]-0.7
                navigate(x=0, y=diff, z=0, frame_id='navigate_target')
                wait_arrival()
                data=rospy.wait_for_message('/lidar',LaserScan)
                continue
            break
    else: 
        while data.ranges[540]<0.6 and data.ranges[540]>0.8: 
            if (data.ranges[540]<0.6):
                diff=0.7-data.ranges[540]
                navigate(x=0, y=diff, z=0, frame_id='navigate_target')
                wait_arrival()
                data=rospy.wait_for_message('/lidar',LaserScan)
                continue
            if (data.ranges[540]>0.8):
                diff=data.ranges[540]-0.7
                navigate(x=0, y=diff, z=0, frame_id='navigate_target')
                wait_arrival()
                data=rospy.wait_for_message('/lidar',LaserScan)
                continue
            break
    
    print('\nside_function terminated\n')

#The below function tells the drone to move forward.
def find_corresponding_index(a):
    angle_change=90-a
    if (globals()['is_left']):
        index_shift=360-(angle_change*2)
    else:
        index_shift=360+(angle_change*2)
    return index_shift
def front_side(): 
    navigate(x=abs(globals()['front']),y=0,z=0,speed=0.7,frame_id='navigate_target')
    wait_arrival()
    globals()['front']=0

def joker_function():
    rospy.sleep(1)
    data=rospy.wait_for_message('/lidar',LaserScan) #reading the laserscan data.. 
    angle=75
    globals()['side']=data.ranges[globals()['side_index']]
    while (angle>=15 and data.ranges[360]>1 and abs(globals()['side']-data.ranges[globals()['side_index']])<0.5):
        experimental_hyp=abs(data.ranges[find_corresponding_index(angle)])
        theoritical_hypo=abs(data.ranges[globals()['side_index']]/math.cos(math.radians(angle)))
        difference=abs(round(experimental_hyp-theoritical_hypo,3))
        globals()['front']=data.ranges[find_corresponding_index(angle)]*math.sin(math.radians(angle))
        if (difference<0.3 and data.ranges[360]>globals()['front']+1):
            print('hypo is', experimental_hyp)
            print('experimental distance is', data.ranges[find_corresponding_index(angle)])
            print('angle is: ', angle)
            print("safe to move forward :), safe distance is: ", globals()['front'] )
            rospy.sleep(0.1)
            front_side()
            rospy.sleep(0.1)
        else: 
            print('not safe to move %d forward: ',data.ranges[find_corresponding_index(angle)] )
            print ('angle is ', angle)
            angle=angle-15
        data=rospy.wait_for_message('/lidar',LaserScan)

def take_stop_action():
    print('\nstop function started\n')
    data=rospy.wait_for_message('/lidar',LaserScan) #reading the laserscan data..
     #case 1: stop if inner corner
    if (data.ranges[360]<1.5 and abs(data.ranges[180]-globals()['side'])<0.5):
        print('\nleft inner corner\n')
        navigate(yaw=math.radians(-90),frame_id='navigate_target', auto_arm=True)
        wait_arrival()
        print('\ninner corner detected, rotate to the right\n')
        globals()['is_left']=True
        print('\nusing the left side to navigate\n')
        globals()['side_index']=180
        return None
    elif (data.ranges[360]<1.5 and abs(data.ranges[540]-globals()['side'])<0.5):
        print('\nright inner corner\n')
        navigate(yaw=math.radians(90),frame_id='navigate_target', auto_arm=True)
        wait_arrival()
        print('\ninner corner detected, rotate to the left\n')
        print('\nusing the right side to navigate\n')
        globals()['is_left']=False
        globals()['side_index']=540
        return None
    
     #case 2: stop if outer corner
    else: 
        print('\nouter corner is detected\n')
        current_side=data.ranges[globals()['side_index']]
        while (abs(current_side-globals()['side'])<0.5 and data.ranges[360]>1):
            navigate(x=0.5,y=0,z=0, speed=0.7,frame_id='navigate_target')
            wait_arrival()
            data=rospy.wait_for_message('/lidar',LaserScan)
            current_side=data.ranges[globals()['side_index']]
        navigate(x=0.5,y=0,z=0, speed=0.7,frame_id='navigate_target')
        wait_arrival()
        if (globals()['is_left']):
            navigate(yaw=math.radians(90),frame_id='navigate_target', auto_arm=True)
            wait_arrival()
        else:
            navigate(yaw=math.radians(-90),frame_id='navigate_target', auto_arm=True)
            wait_arrival()
        navigate(x=globals()['side']+1,y=0,z=0, speed=0.7, frame_id='navigate_target')
        wait_arrival()
        #evaluate_situation()
    print('\nstop function terminated\n')
def evaluate_situation(): 
    data=rospy.wait_for_message('/lidar',LaserScan)
    if(data.ranges[180]>=data.ranges[540]):
        globals()['is_left']=False
        globals()['side_index']=540
        print('right logic')

    else:
        globals()['is_left']=True
        globals()['side_index']=180
        print('left logic')

#Takeoff first..
print('\n taking off\n') 
navigate(x=0, y=0, z=1.5, auto_arm=True, frame_id='body')
wait_arrival()
while(1):
    evaluate_situation()
    side_function()
    rospy.sleep(0.2)
    joker_function()
    rospy.sleep(0.2)
    take_stop_action()
    rospy.sleep(0.2)
    globals()['front']=0
    globals()['side']=0

    