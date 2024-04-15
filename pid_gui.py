import time
import sys
import re
import asyncio
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


uploadmission('/home/yash/Downloads/random2.waypoints')


#takeoff
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
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    master.param_fetch_one(param_name)
    param_value_msg = master.recv_match(type='PARAM_VALUE', blocking=True)
    param_value = param_value_msg.param_value
    return param_value


#start automission
master.mav.command_long_send(master.target_system, master.target_component,
                                300 , 0, 0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

tms=500
def update():
    a=pread('ATC_RAT_RLL_P')
    rollp.config(text=round(a,6))
    b=pread('ATC_RAT_RLL_I')
    rolli.config(text=round(b,6))
    c=pread('ATC_RAT_RLL_D')
    rolld.config(text=round(c,6))
    d=pread('ATC_RAT_PIT_P')
    pitchp.config(text=round(d,6))
    e=pread('ATC_RAT_PIT_I')
    pitchi.config(text=round(e,6))
    f=pread('ATC_RAT_PIT_D')
    pitchd.config(text=round(f,6))
    g=pread('ATC_RAT_YAW_P')
    yawp.config(text=round(g,6))
    h=pread('ATC_RAT_YAW_I')
    yawi.config(text=round(h,6))
    i=pread('ATC_RAT_YAW_D')
    yawd.config(text=round(i,6))
    root.after(tms, update)
    

root = Tk()
def hello():
    print("hello")
root.title('PID Tuning')
root.geometry('700x400')

frame1=Frame(root)
frame1.pack()

frame2=Frame(root)
frame2.pack()

frame3=Frame(root)
frame3.pack()

w = Label(frame1, text='Enter rate change percentage:')
w.grid(row=0,column=0)

def button_func(pname, entry_string):
    print(pname)
    print(entry_string.get())
    perc=float(entry_string.get())/100
    print(f'percentage={perc}')
    read=(pread(pname))
    setval=read*(1+perc)
    print(f"{pname}={read}")
    print(f'newvalue={setval}')
    master.mav.param_set_send(
    master.target_system,
    master.target_component,
    pname.encode('utf-8'),  # Parameter name
    setval,  # Custom rate value
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16  # Parameter type
    )

def exitgui():
    root.destroy()

entry_string = StringVar(value = '0')
e=Entry(frame1, cursor='circle', textvariable = entry_string, width= 5)
e.grid(row=0,column=1)

Label(frame2, text='Rate Roll', bg='green').grid(row=0,column=0,padx=10,pady=10) #roll
Label(frame3, text='P').grid(row=0,column=0, padx=3)
rollp=Button(frame3, command = lambda: button_func('ATC_RAT_RLL_P',entry_string))
rollp.grid(row=0,column=1, padx=15)
Label(frame3, text='I').grid(row=1,column=0, padx=3)
rolli=Button(frame3, command = lambda: button_func('ATC_RAT_RLL_I',entry_string))
rolli.grid(row=1,column=1, padx=15)
Label(frame3, text='D').grid(row=2,column=0, padx=3)
rolld=Button(frame3, command = lambda: button_func('ATC_RAT_RLL_D',entry_string))
rolld.grid(row=2,column=1, padx=15)

Label(frame2, text='Rate Pitch', bg='green').grid(row=0,column=1,padx=10,pady=10) #pitch
Label(frame3, text='P').grid(row=0,column=2, padx=3)
pitchp=Button(frame3, command = lambda: button_func('ATC_RAT_PIT_P',entry_string))
pitchp.grid(row=0,column=3, padx=15)
Label(frame3, text='I').grid(row=1,column=2, padx=3)
pitchi=Button(frame3, command = lambda: button_func('ATC_RAT_PIT_I',entry_string))
pitchi.grid(row=1,column=3, padx=15)
Label(frame3, text='D').grid(row=2,column=2, padx=3)
pitchd=Button(frame3, command = lambda: button_func('ATC_RAT_PIT_D',entry_string))
pitchd.grid(row=2,column=3, padx=15)

Label(frame2, text='Rate Yaw', bg='green').grid(row=0,column=2,padx=10,pady=10) #pitch
Label(frame3, text='P').grid(row=0,column=4, padx=3)
yawp=Button(frame3, command = lambda: button_func('ATC_RAT_YAW_P',entry_string))
yawp.grid(row=0,column=5, padx=15)
Label(frame3, text='I').grid(row=1,column=4, padx=3)
yawi=Button(frame3, command = lambda: button_func('ATC_RAT_YAW_I',entry_string))
yawi.grid(row=1,column=5, padx=15)
Label(frame3, text='D').grid(row=2,column=4, padx=3)
yawd=Button(frame3, command = lambda: button_func('ATC_RAT_YAW_D',entry_string))
yawd.grid(row=2,column=5, padx=15)

exit=Button(frame3, text='EXIT', command = lambda: exitgui())
exit.grid(row=3,column=5, padx=15)

update()
root.mainloop()

#rtl
master.mav.command_long_send(
master.target_system,
master.target_component,
20, 0, 0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
