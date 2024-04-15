import time
import sys
import re
from tkinter import *

# Import mavutil
from pymavlink import mavutil
from pymavlink import mavwp

connection_string='udp:127.0.0.1:14551'

# Create the connection
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

def default(pname,value):
    master.mav.param_set_send(
    master.target_system,
    master.target_component,
    pname.encode('utf-8'),  # Parameter name
    value,  # Custom rate value
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16  # Parameter type
    )

ch3=-1
ch3=int(input("Press 1 to load default pid values: "))
if (ch3):
    default('ATC_RAT_RLL_P',0.135)
    default('ATC_RAT_RLL_I',0.135)
    default('ATC_RAT_RLL_D',0.0036)
    default('ATC_RAT_PIT_P',0.135)
    default('ATC_RAT_PIT_I',0.135)
    default('ATC_RAT_PIT_D',0.0036)
    default('ATC_RAT_YAW_P',0.180)
    default('ATC_RAT_YAW_I',0.018)
    default('ATC_RAT_YAW_D',0.000)
else:
    pass

#flight mode 5(loiter)
time.sleep(1)
master.wait_heartbeat()
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    176, 0, 1, 5, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

time.sleep(2)
# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    400, 0, 1, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
master.motors_armed_wait()
print('Armed!')

#flight mode 4(guided)
time.sleep(2)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    176, 0, 1, 4, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

#load waypoint file
wp = mavwp.MAVWPLoader()

def cmd_set_home(home_location, altitude):
    print('--- ', master.target_system, ',', master.target_component)
    master.mav.command_int_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, # default 0
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        int((home_location[0])*10**7), # lat
        int((home_location[1])*10**7), # lon
        0.000000)

tfound=0; rtlfound=0
def uploadmission(aFileName):
    home_location = None
    home_altitude = None
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                ln_z=float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                if(ln_command==22):
                    global tfound
                    tfound=1
                if(ln_command==20):
                    global rtlfound
                    rtlfound=1
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                                             
    cmd_set_home(home_location,home_altitude)
    msg = master.recv_match(type = 'COMMAND_ACK',blocking = True)
    print(msg)
    print('Set home location: {0} {1} {2}'.format(home_location[0],home_location[1],home_altitude))

    time.sleep(1)

    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        print('executed')
        msg = master.recv_match(type='MISSION_REQUEST',blocking=True)
        print(msg)
        master.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))


uploadmission('/home/yash/Downloads/randomTRTL.waypoints')

#takeoff
if(tfound==0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0,
        13.0302279,
        77.5665213,
        10)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

def pread(param_name):
    try:
        # Create a MAVLink connection
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
        # Request the specific parameter
        master.param_fetch_one(param_name)

        # Wait for the parameter value to be received
        param_value_msg = master.recv_match(type='PARAM_VALUE', blocking=True)

        # Print the parameter name and value
        if param_value_msg is not None:
            param_value = param_value_msg.param_value
            #print(f"{param_name}: {param_value}")
        else:
            print(f"Parameter '{param_name}' not found.")

    except Exception as e:
        print(f"An error occurred: {e}")
    return param_value

'''def pread(param_name):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1)
    param = master.recv_match(type='PARAM_VALUE', blocking=True)
    #print(f"{param.param_id}: {param.param_value}")
    return param.param_value'''

def pwrite(pname,value):
    print(f"Type:{type}")
    print(f"{pname}")
    print(f"old value={pread(pname)}")
    if(type=='i'):
        setval=pread(pname)*(1+(value/100))
    elif(type=='d'):
        setval=pread(pname)*(1-(value/100))    
    master.mav.param_set_send(
    master.target_system,
    master.target_component,
    pname.encode('utf-8'),  # Parameter name
    setval,  # Custom rate value
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16  # Parameter type
    )
    print(f"%={value/100}")
    print(f"new value={pread(pname)}")

#start automission
master.mav.command_long_send(master.target_system, master.target_component,
                                300 , 0, 0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


ch='y'
while(ch=='y'):
    print(f"1.ROLL P= {round(pread('ATC_RAT_RLL_P'),6)}",end='\t\t')
    print(f"2.ROLL I= {round(pread('ATC_RAT_RLL_I'),6)}",end='\t\t')
    print(f"3.ROLL D= {round(pread('ATC_RAT_RLL_D'),6)}",end='\n')
    print(f"4.PITCH P= {round(pread('ATC_RAT_PIT_P'),6)}",end='\t\t')
    print(f"5.PITCH I= {round(pread('ATC_RAT_PIT_I'),6)}",end='\t\t')
    print(f"6.PITCH D= {round(pread('ATC_RAT_PIT_D'),6)}",end='\n')
    print(f"7.YAW P= {round(pread('ATC_RAT_YAW_P'),6)}",end='\t\t')
    print(f"8.YAW I= {round(pread('ATC_RAT_YAW_I'),6)}",end='\t\t')
    print(f"9.YAW D= {round(pread('ATC_RAT_YAW_D'),6)}",end='\n')
    ch2 = int(input('Enter value to change: '))
    value = float(input("Percentage: "))
    global type
    type=(input("Press i to increase or d to decrease"))
    if(ch2==1):
        pwrite('ATC_RAT_RLL_P',value)
    elif(ch2==2):
        pwrite('ATC_RAT_RLL_I',value)
    elif(ch2==3):
        pwrite('ATC_RAT_RLL_D',value)
    elif(ch2==4):
        pwrite('ATC_RAT_PIT_P',value)
    elif(ch2==5):
        pwrite('ATC_RAT_PIT_I',value)
    elif(ch2==6):
        pwrite('ATC_RAT_PIT_D',value)
    elif(ch2==7):
        pwrite('ATC_RAT_YAW_P',value)
    elif(ch2==8):
        pwrite('ATC_RAT_YAW_I',value)
    elif(ch2==9):
        pwrite('ATC_RAT_YAW_D',value)
    else:
        print("Enter correct option")
    ch=input("stay?")

#rtl
if(rtlfound==0):
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    20, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)