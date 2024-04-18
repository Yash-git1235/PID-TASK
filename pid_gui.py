import time
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

def setdefault():
    default('ATC_RAT_RLL_P',0.135)
    default('ATC_RAT_RLL_I',0.135)
    default('ATC_RAT_RLL_D',0.0036)
    default('ATC_RAT_PIT_P',0.135)
    default('ATC_RAT_PIT_I',0.135)
    default('ATC_RAT_PIT_D',0.0036)
    default('ATC_RAT_YAW_P',0.180)
    default('ATC_RAT_YAW_I',0.018)
    default('ATC_RAT_YAW_D',0.000)

ch3=-1
ch3=int(input("Press 1 to load default pid values: "))
if (ch3):
    setdefault()
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
        t=0
        size=len(f.readlines())
        f.seek(0)
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(linearray[0])+t
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
                if(i==1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                if(i==2 and ln_command!=22): #check for takeoff cmd in wp file
                    t=1
                elif(t==1 and i==3): #add takeoff cmd if not present
                    p_tkf = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq-1, ln_frame, 22,
                                                                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, home_location[0], home_location[1], ln_z)
                    wp.add(p_tkf)
                    print("Takeoff command added.")
                    p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,ln_command,
                                                                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                    wp.add(p)
                elif(i==size-1 and ln_command!=20): #check and add RTL cmd if not present
                    p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,ln_command,
                                                                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                    wp.add(p)
                    ln_command=20
                    p_rtl = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq+1, ln_frame, ln_command,
                                                                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, home_location[0], home_location[1], 0)
                    wp.add(p_rtl)
                    print("RTL command added.")
                else: #add wp
                    p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,ln_command,
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


uploadmission('your_waypoint_file_here')

def pread(param_name):
    #master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    master.param_fetch_one(param_name)
    param_value_msg = master.recv_match(type='PARAM_VALUE', blocking=True)
    param_value = param_value_msg.param_value
    return param_value

#start automission
time.sleep(1)
master.wait_heartbeat()
master.mav.command_long_send(master.target_system, master.target_component,
                                300 , 0, 0, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

tms=500 #refresh rate (ms)
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
root.title('PID Tuning')
root.geometry('500x250')

frame1=Frame(root)
frame1.pack()

frame2=Frame(root)
frame2.pack()

frame3=Frame(root)
frame3.pack()

frame4=Frame(root)
frame4.pack(side='right')

w = Label(frame1, text='Enter rate change percentage:')
w.grid(row=0,column=0)

def button_func(pname, entry_string, type):
    print(pname)
    perc=float(entry_string.get())/100
    print(f'percentage={perc}')
    read=(pread(pname))
    print(f"old value={read}")
    n=1
    if(type==1):
        if(read==0):
            read=1
            n=0
        setval=read*(n+perc)
    elif(type==-1):
        setval=read*(1-perc)
    print(f'new value={setval}')
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

Label(frame2, text='Rate Roll', bg='green').grid(row=0,column=0,padx=50,pady=10) #roll
Label(frame3, text='P').grid(row=0,column=0, padx=3)
rollp=Label(frame3)
rollp.grid(row=0,column=1)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_RLL_P',entry_string,1)).grid(row=0,column=2)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_RLL_P',entry_string,-1)).grid(row=0,column=3)
Label(frame3, text='I').grid(row=1,column=0, padx=3)
rolli=Label(frame3)
rolli.grid(row=1,column=1)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_RLL_I',entry_string,1)).grid(row=1,column=2)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_RLL_I',entry_string,-1)).grid(row=1,column=3)
Label(frame3, text='D').grid(row=2,column=0, padx=3)
rolld=Label(frame3)
rolld.grid(row=2,column=1)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_RLL_D',entry_string,1)).grid(row=2,column=2)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_RLL_D',entry_string,-1)).grid(row=2,column=3)

Label(frame2, text='Rate Pitch', bg='green').grid(row=0,column=1,padx=50,pady=10) #pitch
Label(frame3, text='P').grid(row=0,column=4, padx=3)
pitchp=Label(frame3)
pitchp.grid(row=0,column=5)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_PIT_P',entry_string,1)).grid(row=0,column=6)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_PIT_P',entry_string,-1)).grid(row=0,column=7)
Label(frame3, text='I').grid(row=1,column=4, padx=3)
pitchi=Label(frame3)
pitchi.grid(row=1,column=5)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_PIT_I',entry_string,1)).grid(row=1,column=6)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_PIT_I',entry_string,-1)).grid(row=1,column=7)
Label(frame3, text='D').grid(row=2,column=4, padx=3)
pitchd=Label(frame3)
pitchd.grid(row=2,column=5)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_PIT_D',entry_string,1)).grid(row=2,column=6)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_PIT_D',entry_string,-1)).grid(row=2,column=7)

Label(frame2, text='Rate Yaw', bg='green').grid(row=0,column=2,padx=50,pady=10) #yaw
Label(frame3, text='P').grid(row=0,column=8, padx=3)
yawp=Label(frame3)
yawp.grid(row=0,column=9)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_YAW_P',entry_string,1)).grid(row=0,column=10)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_YAW_P',entry_string,-1)).grid(row=0,column=11)
Label(frame3, text='I').grid(row=1,column=8, padx=3)
yawi=Label(frame3)
yawi.grid(row=1,column=9)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_YAW_I',entry_string,1)).grid(row=1,column=10)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_YAW_I',entry_string,-1)).grid(row=1,column=11)
Label(frame3, text='D').grid(row=2,column=8, padx=3)
yawd=Label(frame3)
yawd.grid(row=2,column=9, padx=15)
Button(frame3, text='+', command = lambda: button_func('ATC_RAT_YAW_D',entry_string,1)).grid(row=2,column=10)
Button(frame3, text='-', command = lambda: button_func('ATC_RAT_YAW_D',entry_string,-1)).grid(row=2,column=11)

reset=Button(frame4, text='RELOAD DEFAULTS', command = lambda: setdefault())
reset.grid(row=0, column=0, pady=20)

exit=Button(frame4, text='EXIT', command = lambda: exitgui())
exit.grid(row=0,column=1, pady=20, padx=40)

update()
root.mainloop()
