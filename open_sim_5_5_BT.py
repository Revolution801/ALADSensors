#!/usr/bin/env python
# For gazebo sim, mavros, and DJI
# creates vector in direction of desired travel.
import rospy
import math
from MovingAverageFilter import MAFAngles
import numpy as np
import genpy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from collections import namedtuple
from lpf import lpf

#Sensor dependencies
from smbus2 import SMBus
import time
import serial
from datetime import datetime

#Sensor initialization
print "Opening Log file and initializing sensors"
i2c_addr = 0x44
result_reg = 0x00
config_reg = 0x01
config = 0b1100111000000000 
try:
	ser = serial.Serial('/dev/sqm', 115200, timeout=1)
except:
	ser = serial.Serial('/dev/_sqm', 115200, timeout=1)

data_file_name = "../../../home/odroid/datalogs/sensor_data.dat"
data_file = open(data_file_name, "a")
data_file.write("YYYY-MM-DD HH:mm:ss.ffffff,lux,mag/arcsec^2,Hz,Counts,Period,Celcius,lat,lon,alt\n")
i2c = SMBus(1)
i2c.write_i2c_block_data(i2c_addr, config_reg, [0b11001110, 0b00000000])
print "Done initializing Sensors"



# mode = 1 -> sim
# mode = 2 -> dji
# mode = 3 -> mavros
mode = rospy.get_param("mode")
bag = rospy.get_param("bag")
agent = rospy.get_param("AGENT_NUMBER")
print agent

run_open_sim = "lawnMower"

if mode == 1: # sim
    from sensor_msgs.msg import NavSatFix
    ext_lpf = lpf()
    agent = int(agent)
elif mode == 2: # dji
    from geographic_msgs.msg import GeoPoint
    from sensor_msgs.msg import Range
    import dji_sdk.msg
    ext_lpf = lpf(dt=.1)
    agent = int(agent)
else: # mavros
    from geographic_msgs.msg import GeoPoint
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import Range
    from sensor_msgs.msg import NavSatFix
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt8
    import enif_iuc.msg
    from enif_iuc.msg import AgentSource
    
    agent = int(agent)
    ext_lpf = lpf(tc=2.75, dt=0.05)
    dterm_lpf = lpf(tc=2, dt=0.05)
    yawR_lpf = lpf(tc=3.5, dt=0.02)

past_position = []
pp = 0
ll = 0
mavState=0

vel_flag = True
z_desired = 2
past_z_err = 0
past_pos_x_err = 0
past_pos_y_err = 0

pf = [0,0] #[angle,mag]
#open_sector = []
open_sector_live = [] # open sector live is changed every scan, open_sector is set in cmd and odes not change during cmd
avoid_vector = []
xy = namedtuple("xy","x y")
past_choices = []
past_choice = xy(0, 0)
past_choices += [past_choice]
gg = 0
heading = 0
pos_flag = 0
MyStruct = namedtuple("Mystruct", "start stop narrow start_dis stop_dis")
MyStruct2 = namedtuple("Mystruct2", "start stop narrow start_dis stop_dis min_start_dis min_stop_dis")
lat_lon = [0,0]
alt = 0

t_m = 1
joy_target = [0,0]
min_dis = 2.0   # minimum distance to stay away from object for open sector
max_dis = 7.0   # max distance considered for open sector !!!! most be change down
max_scan_dis = 30  # max distance
vel_x = 0 # used in cmd for step cmd check
vel_y = 0
yaw = 0  # needed global b/c vel to heading in global during flight. velocity callback

waypoint = [0,10]
hold_flag = 0
heading_nxt_wp = 0
pos_z = 0
nearest_dis = 0 # used in pf to change speed
past_choice_full_flag = False
ranges = []
angle_increment = 0
filter_cmd = MAFAngles(30)
x_o = None
old_time = 0 # used in avoiding other quad for random var change
rand_add = [0,0] # same as above
# for swarm
current_xy = [0,0]
agent_location = [[None,None]]*10


use_pf = False
sin = []
cos = []

# varts for avoding timeout
zero_vel_flag = False
close_timeout_flag = False
time = 0



# Flags
#increase_speed_if_open_ahead = True # ONLY WORKS WITH pot_field_only == False
add_past                     = True
pot_field_only               = False
avoid_past                   = False # also adds past in open sector TODO fix
head_for_tru_if_open         = False # does not get out of large traps as well
avoid_rad                    = 5 # meters
z_cmd = 0
msg_error=Twist()

def set_global_vel(msg):
    global mode, heading, yaw, vel_x, vel_y
    #print "set gloabl"
    if mode == 1:
        vel_x =  msg.vector.x # inertial frame
        vel_y =  msg.vector.y
    elif mode == 2:
        vel_x = msg.vx  # inertial frame
        vel_y = -msg.vy
    else:
        vel_x = msg.twist.linear.y
        vel_y = -msg.twist.linear.x  # inertial frame

    heading = math.atan2(vel_y, vel_x) #global frame

def set_lat_lon(msg):
    if mode == 1: # sim
        global lat_lon
        lat_lon = [msg.latitude, msg.longitude]
    else: # mavros and dji
        global lat_lon, pos_flag, pos_z_fc, x_o
        lat_lon = [msg.latitude, msg.longitude]
        alt = msg.altitude
        if x_o is None:
           x_o = 40.80261 #msg.latitude
        if x_o is not None:
            pub_xo(x_o)
        pos_flag = 1

def pub_xo(x):
    geo = GeoPoint()
    geo.latitude = x
    geo.longitude = 0
    geo.altitude = 0
    pub_geo = rospy.Publisher('xo', GeoPoint, queue_size=1)
    pub_geo.publish(geo)

def distSensor_callback(msg):
    global ext_height, ext_lpf, pos_z, z_cmd, past_z_err, msg_error, dterm_lpf
    if mode == 1: # sim
        ground_range = msg
    else: # mavros and DJI
        ground_range = msg.range

    if ground_range > 50:
        ext_lpf.update(50)
    elif ground_range < 0:
        ext_lpf.update(0)
    else:
        ext_lpf.update(ground_range)

    pos_z = ext_lpf.lastOut
    z = pos_z

    # ---- Altitude control (PD) ---- #
    if mode == 3: # mavros
        alt_gain = 0.75
        alt_Dgain = 10
        z_err = z_desired - z
        dterm_lpf.update(z_err-past_z_err);        
        z_cmd =z_err*alt_gain + dterm_lpf.lastOut*alt_Dgain
        #pd(z_err, past_z_err, alt_gain, alt_Dgain)
        msg_error.linear.x=z_err;
        msg_error.linear.y=dterm_lpf.lastOut;

        past_z_err = z_err
        # cap z linear cmd
        if z_cmd > 3:  # max accent
            z_cmd = 3
        elif z_cmd < -2:  # max decent
            z_cmd = -2
            
    elif mode == 2 or mode == 1:  # dji and sim
        alt_gain = 1
        alt_Dgain = 10
        z_err = z_desired - z
        z_cmd = pd(z_err, past_z_err, alt_gain, alt_Dgain)
        past_z_err = z_err
        # cap z linear cmd
        if z_cmd > 1:  # max accent
            z_cmd = 1
        elif z_cmd< -.75:  # max decent
            z_cmd = -.75

    if mode != 1:  # DJI and Mavros
        pub = rospy.Publisher("filtered_ext_height", Range, queue_size=1)
        pubSonar = msg
        pubSonar.range = pos_z
        pub.publish(pubSonar)

def main_cmd(msg): # gets pos and runs command function
    #r = rospy.Rate(35)
    #r.sleep()

    global yaw, current_xy, pos_z, run_open_sim
    print "run_open_sim value: "
    print run_open_sim
    if run_open_sim != "info":
    
        if mode == 1:
            r = rospy.Rate(40)
            r.sleep()
            position = msg.pose.position
            quat = msg.pose.orientation
            quaternion = [quat.x, quat.y, quat.z, quat.w]
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            lat = position.x
            lon = position.y
            current_xy = [lat,lon]
            distSensor_callback(position.z)
            command(lat, lon, pos_z)
        else:
            global pos_flag, lat_lon
            if mode == 2:
                quaternion = [msg.q3, msg.q1, msg.q2, msg.q0]  # switching the order
                euler = tf.transformations.euler_from_quaternion(quaternion)
                yaw = -euler[0]
            else: # Mavros
                quaternion = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
                euler = tf.transformations.euler_from_quaternion(quaternion)
                yaw = -euler[0] + math.pi / 2.0
            print "pos_flag value: "
            print pos_flag
            if pos_flag == 1: # only is a new  gps location is read will cmd run
                pos_flag = 0
                temp_xy = latlon_2_m(lat_lon[0], lat_lon[1])
                current_xy = [temp_xy[0], temp_xy[1]]
                command(temp_xy[0], temp_xy[1], pos_z)

def state_sub(msg):
    global mavState
    mavState=msg.data;

def source_sub(msg):
    global agent_location
    agent_location[9] = latlon_2_m(msg.source.latitude, msg.source.longitude)
    
                
def read_sonar(msg):
    global sonar_dis
    sonar_dis = list(msg.ranges)

def process_laser(header):
    global hold_flag, sonar_dis
    global ranges, angle_increment, pot_field_only
    global use_pf, pf, run_open_sim

    if run_open_sim!='info':
        ranges = list(header.ranges)
        angle_increment = header.angle_increment
        angle_min = header.angle_min
        #ranges = fill_blind_spot(ranges, angle_min,header.angle_max, angle_increment)
        #if sim == 1:
        #    sonar_dis = get_sonar(ranges, angle_increment)
        #ranges = consider_sonar(ranges, angle_increment, sonar_dis, 0) # if 1, does not consider laser
        #ranges = segment(ranges, angle_increment)
        if hold_flag == 0 and pot_field_only != True and not use_pf:
            #print "FIND open sectors"
            global open_sector_live, avoid_vector
            [open_sector_live, avoid_vector] = open_sectors(ranges, angle_increment) #global
            pf = [0,0]
        else:
            open_sector_live = []
            #print 'opensectors', open_sector_live
        if len(open_sector_live) == 0:
            potential_field(ranges, angle_increment)
            #pub_new_laser(ranges, header)

def set_waypoint(msg):
    global waypoint, hold_flag, heading_nxt_wp, z_desired, pos_z, run_open_sim

    if run_open_sim!="info":
        if mode == 1:
            global t_m
            waypoint = [msg.linear.x, msg.linear.y]
            
            z_desired = msg.linear.z
            hold_flag = msg.angular.x
            #print hold_flag
            t_m = msg.angular.y
            if hold_flag == 1:
                #print "rec hold pos"
                heading_nxt_wp = norm_2pi(msg.angular.z)
            pub_waypoint(waypoint[0], waypoint[1], z_desired)
        else: # mavros and DJI
            waypoint = latlon_2_m(msg.latitude, msg.longitude)
            if mode == 2: # DJI
                z_desired = msg.altitude
            else: # mavros
                z_desired = msg.target_height
            if msg.staytime != 0:
                hold_flag = 1
                heading_nxt_wp = -msg.heading * math.pi/180   # degrees to radians, switch y  axis
            else:
                hold_flag = 0
            pub_waypoint(msg.latitude, msg.longitude, pos_z)
                    

def set_t_m(msg): # DJI and Mavros
    # In sim the t_m is set with the waypoint
    global t_m
    if mode == 2: # DJI
        t_m = msg.idle_velocity
    else:
        t_m = msg.velocity

def callback_avoid_agents(msg):
    global yaw, agent_location, zero_vel_flag, close_timeout_flag, time, run_open_sim
    if run_open_sim!="info":
        if mode == 1:
            # get agent number
            agent = int(msg.header.frame_id)
            # update agent location
            agent_location[agent] = [msg.pose.position.x, msg.pose.position.y]
        else:
            # get agent number
            agent = msg.agent_number
            # update agent location
            agent_location[agent] = latlon_2_m(msg.mps.GPS_latitude, msg.mps.GPS_longitude)
        print "agent_location", agent_location





# def pf_quad():
#     global agent_location
#     global old_time, rand_add
#     add_to_pf_x = 0
#     add_to_pf_y = 0
#     print "agent_locations", agent_location
#     for agent in agent_location:
#         # pf constants
#         s = 5  # 3 6/5
#         a = 1
#         if agent[0] is not None: # for every other agent
#             x = agent[0] # agent location
#             y = agent[1]
#             vec = np.array([x - current_xy[0], y - current_xy[1]]) # vector from this anent towards other agent
#             M = np.linalg.norm(vec) # magnitude of vector
#             dis = M - avoid_rad + 2 # subtract the avoid radius and add how much pf stays away on it own
#             if dis <= 0:  # check for negative dis
#                 dis = .001
#             avoid = -vec * (1 / M) * (s / (dis ** a))  # avoid vector
#             angle_body = math.atan2(avoid[1], avoid[0]) - yaw  # change to body frame
#
#             mag = np.linalg.norm(avoid) - 0.5 # to modify how pf behaves check matlab code cal_avoid_quads_pf.m
#             if mag < 0:
#                 mag = 0
#             if mag > 2:
#                 mag = 2
#             add_to_pf_x += mag * math.cos(angle_body) # TODO this could add up  mag > 2 needs to be fixed if more then 2 quads
#             add_to_pf_y += mag * math.sin(angle_body)
#     #avoid_all = np.array([add_to_pf_x, add_to_pf_y])
#     #
#     # # Add a random force to avoid minima
#     # if rospy.get_time() - old_time > 1 and np.linalg.norm(avoid_all) > .3 and not hold_flag:
#     #     # if not holding position and pf force is greater then .3 set random force
#     #     old_time = rospy.get_time()
#     #     rand_add = np.random.rand(2)*2
#     # elif np.linalg.norm(avoid_all) < .3 or hold_flag:
#     #     # if force is low or holding position remove random force
#     #     rand_add = np.array([0,0])
#     # avoid_w_rand = np.array([add_to_pf_x, add_to_pf_y]) + rand_add
#     # #print 'rand add', rand_add, avoid_w_rand, [add_to_pf_x, add_to_pf_y]
#     # print "avoid_w_rand", avoid_w_rand
#     pub_marker(math.atan2(add_to_pf_y, add_to_pf_x) - yaw, math.sqrt(add_to_pf_x ** 2 + add_to_pf_y ** 2), 'avoid_quad',
#                [1, .5, .5, 0])
#     return [add_to_pf_x, add_to_pf_y]

def latlon_2_m(lat,lon):
    global x_o
    x_m = 6371000.0 * lat * math.pi / 180
    y_m = -6371000.0 * lon * math.pi / 180 * math.cos(x_o * math.pi / 180.0)
    return [x_m, y_m]



def fill_blind_spot(ranges, angle_min, angle_max, angle_increment):

    global max_scan_dis
    length = len(ranges)
    jj = 0
    for M in ranges:  # replace inf and nan to 5
        if math.isnan(M) or math.isinf(M):
            ranges[jj] = 30
        if ranges[jj] > 30:
            ranges[jj] = 30
        if ranges[jj] < .5:
            ranges[jj] = 30.000
        jj += 1
    r2 = ranges[0]
    r1 = ranges[length - 1]
    angle = angle_min
    # get slope
    x1 = (math.cos(angle_max) * r1)
    y1 = (math.sin(angle_max) * r1)
    x2 = (math.cos(angle_min) * r2)
    y2 = (math.sin(angle_min) * r2)
    if (x2-x1 == 0):
        slope = 1000
    else :
        slope = (y2 - y1) / (x2 - x1)
    b = y2 - slope * x2
    while angle > -3.1415:
        gap_M = [b / (math.sin(angle) - slope * math.cos(angle))]
        ranges = gap_M + ranges
        angle -= angle_increment
    angle = angle_max
    while angle < 3.1415:
        gap_M = [b / (math.sin((angle)) - slope * math.cos(angle))]
        ranges = ranges + gap_M
        angle += angle_increment
    #kk = 0
    #for M in ranges: # changes laser scan distance
    #    if M > max_scan_dis:
    #        ranges[kk] = max_scan_dis
    #    kk += 1
    return ranges

def potential_field(ranges, angle_increment):
    global nearest_dis, sin, cos, t_m

    #### make sin and cos arrays
    if len(sin) < 2:
        angle = 3.1415
        sin = []
        cos = []
        for kk in ranges:
            sin += [math.sin(angle)]
            cos += [math.cos(angle)]
            angle += angle_increment
    ranges_x = np.multiply(ranges,cos)
    ranges_y = np.multiply(ranges,sin)
    ####
    sum_inv_x = 0
    sum_inv_y = 0
    s = 1 # 3 6/5
    a= 5
    angle = 3.1415
    nearest = 30
    for ii in range(0, len(ranges)-1):
        M = ranges[ii]
        if M < nearest:
            nearest = M
        x = ranges_x[ii]
        y = ranges_y[ii]
        sum_inv_x += -x * (1 / M) * (s / (M ** a)) * angle_increment/0.00436332
        sum_inv_y += -y * (1 / M) * (s / (M ** a)) * angle_increment/0.00436332
        angle += angle_increment

    rad_angle = math.atan2(sum_inv_y,sum_inv_x)
    mag = math.sqrt(sum_inv_x**2.0+sum_inv_y**2.0)
    # cap magnitude
    if mag > t_m * 1.5:
        mag = t_m * 1.5
    nearest_dis = nearest## *.4 + 0.6 * nearest_dis
    global pf
    #if pf[0] != 0 and pf[1] != 0:
    #    pf[0] = pf[0]*.5+.5*rad_angle   #gloabl variable used by command
    #    pf[1] = pf[1]*.5+.5*mag   #gloabl variable used by command
    #else:
    pf = [rad_angle, mag]

def norm_2pi(angle):
    while angle < 0:
        if angle < 0:
            angle = angle + 6.28
    while angle > 6.28:
        angle -= 6.28
    return  angle

def norm_pi(angle):
    while angle > 3.14:
        angle -= 6.28
    while angle < - 3.14:
        angle += 6.28
    return angle

def angle_between(start, stop):
    # abs value of angle diff between two values 0 to 2pi start before stop ccw
    if start > 3.14 and stop > 3.14:
        diff = start - stop
    elif start < 3.14 and stop < 3.14:
        diff = start - stop
    else:
        if start >= 3.14:
            start = start - 2*3.14
        diff = - start + stop
    return math.fabs(diff)

def open_sectors(ranges, angle_increment):
    global max_dis, min_dis

    open_angles = []
    m = 0
    avoid_vector = [0, 0]
    closed_sec_min_dis_array = []
    gg = 0
    saved_last = 1

    angle = 3.1415 # TODO fix so open sect works for sectors starting at 3.14 fix needs to be made in angle between function
    while m < len(ranges) - 3:  # TODO over shoots 3.14 at end of ranges same as below ~ 142

        if saved_last == 1:
            closed_sec_min_dis_array += [ranges[m-1]] #if saved last open sector save the corresponding min distance TODO what is this!!!!!

        while (ranges[m] < max_dis):
            #print "ranges closed", ranges[m], 'angle', angle*180/math.pi
            # while distance is less then max distance considered for closed sector
             # TODO does this work if starts closed

            if closed_sec_min_dis_array[gg] > ranges[m]: # check to see if this is the min distance in this closed sector
                closed_sec_min_dis_array[gg] = ranges[m] # if min distance so far save

            angle += angle_increment # increment angle
            if angle > 3.14 * 2: # if angle is is above 2pi reset
                angle = 0

            if ranges[m] < min_dis*.7 : # if range is less then min allowable distance include a repulsive force
                avoid_vector[0] -= (min_dis - ranges[m]) * math.cos(angle)
                avoid_vector[1] -= (min_dis - ranges[m]) * math.sin(angle)
                print "avoid"
                #raise NameError('to close')

            m += 1  # increment through ranges
            if m > len(ranges) - 3:  # break if at end of ranges
                m -= 1
                break

        if ranges[m] >= max_dis: # if range is greater then max_dis then possible open sector
            print 'range at possible open', ranges[m], 'angle', angle*180/math.pi
            dis_at_start_angle = ranges[m-1]
            start = angle # set start of sector
            while (ranges[m] >= max_dis): # cycle though ranges in open sector
                m += 1  # increment through ranges
                angle += angle_increment # increment angle
                if angle > 3.14 * 2: #if angle is above 2pi reset
                    angle = 0
                if m > len(ranges) - 3:  # if at end of ranges break
                    m -= 1
                    break
            stop = angle # once ranges < max_dis set stop angel for open sector
            dis_at_stop_angle = ranges[m]
            angle_diff = angle_between(start, stop) # get the angle between start and stop
            length_of_gap = 1000
            if angle_diff < 3.14:
                x_start = dis_at_start_angle * math.cos(start)
                y_start = dis_at_start_angle * math.sin(start)
                x_stop = dis_at_stop_angle * math.cos(stop)
                y_stop = dis_at_stop_angle * math.sin(stop)
                start_to_stop_x = x_start - x_stop
                start_to_stop_y = y_start - y_stop
                length_of_gap = math.sqrt(start_to_stop_x**2+start_to_stop_y**2)
                print "start angle", start, "stop angle", stop
                print "dis to start", dis_at_start_angle, "dis to stop", dis_at_stop_angle
            print "length of gap", length_of_gap
            print "angle diff", angle_diff
            if length_of_gap > min_dis*2 and angle_diff > .34:
                print "sector SAVED"
                gg += 1  # increment closed sector closest distance array
                saved_last = 1 # sector is larger then min gap and will be saved
                if (length_of_gap > min_dis*2): # if big setctor set narrow to false
                    narrow = 0
                else:
                    narrow = 1 # narrow sector
                open_sect = MyStruct(start, stop, narrow, dis_at_start_angle, dis_at_stop_angle) # save sector struct
                open_angles = open_angles + [open_sect] # add open sector struct to array of open sectors
            else:
                saved_last = 0 # sector was too small and was not saved do not save min dis yet

    closed_sec_min_dis_array.reverse() # reverse min array makes is more convient to cycle through below
    length_closed_array = len(closed_sec_min_dis_array) - 1
    open_angles_with_dist = []
    yy = 0 # used to cycle through closed_sec_min_dis_array
    for kk in open_angles:
        min_start_dis = closed_sec_min_dis_array[length_closed_array - yy] # min distacne of closed sector next to start
        min_stop_dis =  closed_sec_min_dis_array[length_closed_array - 1 - yy] # min distance of closed sector next to stop
        test = [MyStruct2(kk.start, kk.stop, kk.narrow, kk.start_dis, kk.stop_dis, min_start_dis, min_stop_dis)] # make a enw struct woth min dis's
        open_angles_with_dist += test # this could be made in to one line with the one above
        yy += 1
    if len(open_angles_with_dist) > 0: # if there are open sectors add first sector on to end. this is used later
        open_angles_with_dist = open_angles_with_dist + [open_angles_with_dist[0]]
    #print open_angles_with_dist
    return [open_angles_with_dist, avoid_vector]

def find_heading(start, stop, narrow, target, t_in_sector, start_dis, stop_dis, min_start_dis, min_stop_dis):
    # return a unit vector of best heading
    target = norm_2pi(target)
    global min_dis, max_dis
    print "START", start, "stop", stop, "t", target
    pub_marker(start, 7, "start", [.9,0,0,0])
    pub_marker(stop, 7, "stop", [1, 0, 0, 0])
    print "start dis", start_dis, "stop dis", stop_dis, "narrow ", narrow
    print "min start", min_start_dis, "min stop dis", min_stop_dis

    if min_start_dis < min_dis:
        add_to_start = 3.14 / 2 - math.acos(min_dis / max_dis) + 3.14 * (min_dis - min_start_dis) # ToDO what should the added angle be
        print "close take emerg action (start)", add_to_start, min
    else: # start_dis < math.sqrt(min_dis ** 2 + max_dis ** 2) * .8:
        if start_dis<min_dis:
            start_dis=min_dis
        add_to_start = (3.14 / 2 - math.acos(min_dis / start_dis))

        print "possible corner", add_to_start, min_dis, start_dis
    # else:
    #     print 'min_dis, max_dis',min_dis, max_dis
    #     add_to_start = 3.14/2 - math.acos(min_dis / max_dis)

    if min_stop_dis < min_dis:
        add_to_stop = - (3.14 / 2 - math.acos(min_dis / max_dis) + 3.14 * (min_dis - min_stop_dis))
        print "close take emerg action (stop)", add_to_stop

    else: # stop_dis < math.sqrt(min_dis ** 2 + max_dis ** 2) * .8:
        if stop_dis < min_dis:
            stop_dis = min_dis
        add_to_stop =- (3.14/2 - math.acos(min_dis/stop_dis))
        print "possible corner", add_to_stop, min_dis, stop_dis
    # else:
    #     add_to_stop = - (3.14 / 2 - math.acos(min_dis / max_dis))
    #     print "flat wall ?"
    print 'add too start', add_to_start
    print 'add too stop', add_to_stop
    if narrow == 1:
        x_y = [math.cos(start) + math.cos(stop), math.sin(start) + math.sin(stop)]
        # make unit vector
        temp_angle = math.atan2(x_y[1], x_y[0])
        x_y = [math.cos(temp_angle), math.sin(temp_angle)]
        print "narrow"

    elif t_in_sector == 1:
        print "angle between start, target",angle_between(start, target)
        print "angle between target, stop", angle_between(target, stop)
        if angle_between(start, target) > math.fabs(add_to_start) and angle_between(target, stop) > math.fabs(add_to_stop): # free to head straight to target target not to close to start or stop of sector
            x_y = [math.cos(target), math.sin(target)]

            print "free to head for target", x_y
        else: # angle to target is smaller then angle needed to add to start or stop
            ### added 060517
            if (math.fabs(add_to_start) + math.fabs(add_to_stop)) < angle_between(start, stop):
                if closer(target, start, stop) == 1:
                    x_y = [math.cos(start + add_to_start), math.sin(start + add_to_start)]
                    print "target to close to end of sector, but large enough to head for start"
                else:
                    x_y = [math.cos(stop + add_to_stop), math.sin(stop + add_to_stop)]
                    print "target to close to end of sector, but large enough to head for stop"
            ###
            elif math.fabs(add_to_stop) < math.fabs(add_to_start):
                if between(norm_2pi(start + add_to_start), start, stop ):
                    x_y = [math.cos(start + add_to_start), math.sin(start + add_to_start)]
                    print "to narrow of sector add to start "
                else:
                    x_y = [math.cos(stop), math.sin(stop)]
                    print "to narrow add to start is to large go to stop"
            else:
                if between(norm_2pi(stop + add_to_stop), start, stop):
                    x_y = [math.cos(stop + add_to_stop), math.sin(stop + add_to_stop)]
                    print "to narrow add to stop"
                else:
                    x_y = [math.cos(start), math.sin(start)]
                    print "to narrow add to stop is to big go to start"
    else: # target not in sector
        # determine if target is closer to stop or start
        angle_to_start = math.fabs(norm_pi(start - target))
        angle_to_stop = math.fabs(norm_pi(stop - target))
        if angle_to_start < angle_to_stop: # closer to start
            if between(norm_2pi(start + add_to_start), start, stop):
                x_y = [math.cos(start + add_to_start), math.sin(start + add_to_start)]
                print "closer to start"
            else:
                x_y = [math.cos(stop), math.sin(stop)]
            #x_y = [math.cos(start + add_to_start), math.sin(start+ add_to_start)]
                print "closer to start but to much angle to add so go to stop"
        else: # closer to sto
            if between(norm_2pi(stop + add_to_stop), start, stop):
                x_y = [math.cos(stop + add_to_stop), math.sin(stop + add_to_stop)]
                print "closer to stop"
            else:
                x_y = [math.cos(start), math.sin(start)]
            #x_y = [math.cos(stop + add_to_stop), math.sin(stop + add_to_stop)]
                print "closer to stop but to much add so head for start"
    # cycle through and avoid close obstacles

    return x_y

def smaller_angle(ang1,ang2):
    ang1 = norm_2pi(ang1)
    ang2 = norm_2pi(ang2)
    angle_be = ang2 - ang1

    if angle_be > 3.14:
        angle_be = -(6.28 - angle_be)
    if angle_be < -3.14:
        angle_be = (6.28 + angle_be)
    #ang = ang1 + angle_be * w # higher gain less ang1
    return angle_be

def ave_two_angles(ang1,ang2,w):
    ang1 = norm_2pi(ang1)
    ang2 = norm_2pi(ang2)
    angle_be = ang2 - ang1

    if angle_be > 3.14:
        angle_be = -(6.28 - angle_be)
    if angle_be < -3.14:
        angle_be = (6.28 + angle_be)
    ang = ang1 + angle_be * w # higher gain closer to angle 2
    return norm_2pi(ang)

def add_past_choice(target, past_cmd, past_pos_angle=None, past_pos_mag=0): # all global
    global past_choice_full_flag, t_m
    if past_choice_full_flag == True:
        length = len(past_cmd)
        x = 0
        y = 0
        for ii in range(0, length - 1):
            x += past_cmd[ii].x
            y += past_cmd[ii].y
        if length > 0:
            x /= length
            y /= length
        past_cmd_angle = math.atan2(y, x)
        if past_pos_angle is not None: # if avoid past angle is past in
            mash_up = ave_two_angles(past_cmd_angle, past_pos_angle, .3 * past_pos_mag / t_m)
            k_mashup = .6
            pub_marker(mash_up - yaw, t_m, 'mashup', [1, .5, .3, .2], 'laser')
        else:
            mash_up = past_cmd_angle
            k_mashup = .6
        new_target = target + k_mashup * smaller_angle(target, mash_up)
        pub_marker(past_cmd_angle - yaw, t_m, 'all_past_cmd', [1, .1, 0, 1], 'laser')
    else:
        new_target = target

    return norm_2pi(new_target)#past_angle+target #math.atan2(past_gain*y + math.sin(target) , past_gain*x + math.cos(target) )

def make_past_array(x,y):
    global past_position, pp, ll, mode
    num_of_past = 250

    length = len(past_position)

    sum_x = 0
    sum_y = 0
    mag = 1
    mag_reduce = (mag-.7)/(num_of_past)

    hh = 0
    for ii in range(1,length+1):

        vecx = x - past_position[pp-ii].x
        vecy = y - past_position[pp-ii].y
        M = math.sqrt(vecx**2 + vecy**2)
        angle = math.atan2(vecy, vecx)
        sum_x += math.cos(angle)*mag#*.05/M**4
        sum_y += math.sin(angle)*mag#*.05/M**4
        hh += 1
        if hh > 5:
            mag -= mag_reduce
    past_angle = math.atan2(sum_y, sum_x)
    past_mag = math.sqrt(sum_x**2 + sum_y**2)
    if past_mag > t_m:
        past_mag = t_m
    print "avoid past", past_angle, "mag",
    if ll > 5:
        ll = 0
        r = [-1, 1]
        for jj in r:
            for kk in r:
                if pp > (num_of_past - 1):
                    pp = 0
                choice = xy(x + kk, y + jj)

                if len(past_position) <= (num_of_past - 1):
                    past_position += [choice]
                    #print 'pp', len(past_position)
                else:
                    past_position[pp] = choice
                    pp += 1
    ll += 1
    return past_mag, past_angle

def dot(a,b): # angle 1 and angle 2
    A = [math.cos(a), math.sin(a)]
    B = [math.cos(b), math.sin(b)]
    AB = (A[0] * B[0] + A[1] * B[1])/1
    angle = math.fabs(math.acos(AB))
    return angle

def between(target, start, stop):
    # returns 1 if true and 0 if false
    # all inputs need to be from  0 to 2pi
    target = norm_2pi(target)
    start = norm_2pi(start)
    stop = norm_2pi(stop)
    angle_to_stop = stop - target
    angle_to_start = start - target
    if (angle_to_start < 0 and angle_to_stop > 0):
        return 1
    elif angle_to_stop < angle_to_start and (angle_to_stop > 0 and angle_to_start > 0):
        return 1
    elif angle_to_stop < angle_to_start and (angle_to_stop < 0 and angle_to_start < 0):
        return 1
    else:
        return 0

def closer(target, op_1, op_2):
    # return 1 is op_1 is closer to target and 2 is op-2 is closer to target
    angle_to_1 = math.fabs(norm_pi(op_1 - target))
    angle_to_2 = math.fabs(norm_pi(op_2 - target))
    if angle_to_1 < angle_to_2:  # if closer to sector option 1
        return 1
    else:
        return 2

def turn_force(k, diff, sat = -1):
    turn = 0
    if diff > 3.14:
        k = k*(6.28 - diff)
        turn = -1 * k  # turn right
    elif diff < -3.14:
        k = k*(6.28 + diff)
        turn = 1* k  # turn left
    elif 0 < diff < 3.14:
        k = k*diff
        turn = 1* k  # turn left
    elif -3.14 <= diff <= 0:
        k = k*(-diff)
        turn = -1 * k   # turn right
    if sat != -1:
        if turn <= -sat:
            turn = -sat
        elif turn > sat:
            turn = sat
    return turn

def pd(z_err,past_z_err ,alt_gain, alt_Dgain ):
    return (z_err)*alt_gain-(past_z_err-z_err)*alt_Dgain

def log_sensor_data():
    print "Start logging Sensor Data"
    conversion_ready = i2c.read_word_data(i2c_addr, config_reg)
    if((conversion_ready & 0x8000) == 0x8000):
        raw = i2c.read_word_data(i2c_addr, result_reg)
        reading = 0
        reading = reading | (raw & 0x00FF)
        reading = reading << 8
        reading = reading | ((raw & 0xFF00) >> 8)
        lux = 0.01 * (1 << ((reading & 0xF000) >> 12)) * (reading & 0x0FFF)
        try:
            ser.write(b'rx')
            sqm = ser.readline()
            sqm = sqm[3:-2]
        except Exception:
            print e.message
        now = datetime.now()
        data_file.write(str(now))
        data_file.write(',')
        data_file.write(str(lux))
        data_file.write(",")
        data_file.write(str(sqm))
        data_file.write(",")
	data_file.write(str(lat_lon[0]))
	data_file.write(",")
	data_file.write(str(lat_lon[1]))
	data_file.write(",")
	data_file.write(str(pos_z))
        data_file.write('\n')
        print "Finished Logging Sensor Data"

def command(x,y,z):
    global pf, heading, vel_x, vel_y, t_m
    global open_sector_live, avoid_vector
    global past_choice_full_flag
    global gg, past_choice, past_choices
    global min_dis, max_dis
    global yaw, vel_flag
    global waypoint, heading_nxt_wp
    global nearest_dis
    global z_desired, past_z_err, past_pos_x_err,past_pos_y_err
    global past_position, pp, z_cmd
    global use_pf,head_for_tru_if_open
    global zero_vel_flag, close_timeout_flag, time, agent_location
    pos_x_cmd_body = 0  # initialize as 0 incase hold flag is changed while excuting command
    pos_y_cmd_body = 0
    ###### avoiding other quads timem out
    print agent_location
    for a_n, a in enumerate(agent_location):
        if a[1] is not None:
            if agent < a_n:
                print agent, a_n
                print "dis", math.sqrt((current_xy[0] - a[0]) ** 2 + (current_xy[1] - a[1]) ** 2)
                if math.sqrt((current_xy[0] - a[0])**2 + (current_xy[1] - a[1])**2) < 7:

                    if not close_timeout_flag:
                        zero_vel_flag = True
                        close_timeout_flag = True
                        time = rospy.get_time()

            if rospy.get_time() - time > 3:
                close_timeout_flag = False
                zero_vel_flag = False
            elif rospy.get_time() - time > 1:
                zero_vel_flag = False

    ############

    # Initialize twist object for velocity commands
    msg = Twist()

    print "cmd", z_cmd
    msg.linear.z = z_cmd
    # set target location
    t_x_location_global = waypoint[0] #lat [m]
    t_y_location_global = waypoint[1] #long [m]

    # get angle to target
    target_angle_global = math.atan2((t_y_location_global - y), (t_x_location_global - x))
    pub_marker(target_angle_global-yaw, t_m, 'tru_target',[1,1,0,0])

    # --- Calculate distance to wp --- #
    wp_dis = math.sqrt((waypoint[0] - x) ** 2 + (waypoint[1] - y) ** 2)
    print "wp_dis", wp_dis

    # PF radius check
    if wp_dis <  4: #%max_dis*.75:
        use_pf = True
        print "use PF"
    else:
        use_pf = False

    # Damping radius check (slow down near wps)
    if wp_dis < t_m * 1.5:
        dmp_dis = 1
        max_speed = 1
        print "scale to 1 m/s, in dmp dis"
    else:
        dmp_dis = 0
        max_speed = t_m

    # --- Holding pos check --- #
    if hold_flag == 1:
        print "holding position"
        if hold_flag:
            past_position = [] # if holding pos clear past positions
            pp = 0  # counter for saving post positions in array
        vel_flag = True  # reset to check velocity once before yaw control during wp following

        log_sensor_data()
        
        hold_gain = 1
        hold_Dgain = 0

        # calculate error in pos
        pos_x_err = (t_x_location_global - x)
        pos_y_err = (t_y_location_global - y)
        pos_x_cmd = pd(pos_x_err, past_pos_x_err, hold_gain, hold_Dgain)
        pos_y_cmd = pd(pos_y_err, past_pos_y_err, hold_gain, hold_Dgain)
        past_pos_x_err = pos_x_err
        past_pos_y_err = pos_y_err
        # rotate from global to world frame
        pos_x_cmd_body = math.cos(-yaw) * pos_x_cmd - math.sin(-yaw) * pos_y_cmd
        pos_y_cmd_body = math.sin(-yaw) * pos_x_cmd + math.cos(-yaw) * pos_y_cmd
        # cap max velocity cmd while holding to 1
        mag_pos_cmd = math.sqrt(pos_x_cmd_body**2 + pos_y_cmd_body**2)
        if mag_pos_cmd > 1:
            angle_pose_cmd_body = math.atan2(pos_y_cmd_body,pos_x_cmd_body)
            pos_x_cmd_body = math.cos(angle_pose_cmd_body)
            pos_y_cmd_body = math.sin(angle_pose_cmd_body)
        # --- P control on yaw --- #        
        if run_open_sim == "PF":
            yawR_lpf.update(0)
            msg.angular.z = yawR_lpf.lastOut
        else:
            k = 0.5;
            diff = heading_nxt_wp - yaw
            msg.angular.z = turn_force(k, diff, 1)
    else:
        # ---- traveling to wp ---- #
        print "traveling 2 wp"
        diff = heading - yaw
        k = 0.5
        turn = turn_force(k, diff)

        # if speed after hold pos < 0.5 no yaw control
        if math.sqrt(vel_y ** 2.0 + vel_x ** 2.0) < 0.5 and vel_flag == True:
            turn = 0
        elif math.sqrt(vel_y ** 2.0 + vel_x ** 2.0) > 0.5:
            vel_flag = False

        if run_open_sim == "PF":
            yawR_lpf.update(turn)
            msg.angular.z = yawR_lpf.lastOut
        else:
            msg.angular.z = turn

    open_sector = open_sector_live
    # --- Motion Planners used to travel towards wps --- #
    if hold_flag == 0:
        # --- Open Sector (OS) --- #
        tru_target = norm_2pi(target_angle_global - yaw)  # body frame target
        if len(open_sector) != 0 and not use_pf:
            if avoid_past: # boolean set at top avoids past AND IPA
                [avoid_past_mag, avoid_past_angle_g] = make_past_array(x, y) # make vector pointing from past
                pub_marker(avoid_past_angle_g - yaw, avoid_past_mag, "avoid_past", [1, .5, 0 , 1], 'laser')
                target_angle_global = norm_2pi(add_past_choice(target_angle_global, past_choices, avoid_past_angle_g, avoid_past_mag))
            elif add_past: # just adds past choice
                target_angle_global = norm_2pi(add_past_choice(target_angle_global, past_choices))
            # else: # target is unchanged

            target_angle_quad_f = norm_2pi(target_angle_global - yaw)  # body frame virtual target
            print "virtual target angle", target_angle_quad_f

            if head_for_tru_if_open:  # if true check to see if target is in OS
                for sector in open_sector:
                    if between(tru_target, sector.start, sector.stop):
                        target_angle_quad_f = tru_target
                        print "use true target", target_angle_quad_f
            pub_marker(target_angle_global - yaw, t_m, 'past_mod_target', [1, 1, .5, 0])

            x_y = [0, 0]
            ii = 0  # initialize iteration for while loop

            # --- Find closest sector to target and action in sector--- #
            while ii <= len(open_sector) - 2:
                print "start of sector", open_sector[ii].start
                print "end of sector", open_sector[ii].stop

                if between(target_angle_quad_f, open_sector[ii].start, open_sector[ii].stop): # target in sector
                    x_y = find_heading(open_sector[ii].start, open_sector[ii].stop,
                                       open_sector[ii].narrow, target_angle_quad_f, 1, open_sector[ii].start_dis,
                                       open_sector[ii].stop_dis, open_sector[ii].min_start_dis, open_sector[ii].min_stop_dis)
                    print "target in open sector ii"

                if between(target_angle_quad_f, open_sector[ii].stop, open_sector[ii+1].start): # target in closed sector
                    # determine which sector target is closer to
                    if closer(target_angle_quad_f, open_sector[ii].stop, open_sector[ii+1].start) == 1: # closer to [ii]
                        x_y = find_heading(open_sector[ii].start, open_sector[ii].stop, open_sector[ii].narrow,
                                           target_angle_quad_f, 0, open_sector[ii].start_dis, open_sector[ii].stop_dis,
                                           open_sector[ii].min_start_dis, open_sector[ii].min_stop_dis)
                        print "target in closed sector: closer to ii"
                    else: # closer to ii + 1
                        x_y = find_heading(open_sector[ii + 1].start, open_sector[ii + 1].stop,
                                           open_sector[ii + 1].narrow, target_angle_quad_f, 0,
                                           open_sector[ii + 1].start_dis, open_sector[ii + 1].stop_dis,
                                           open_sector[ii + 1].min_start_dis, open_sector[ii + 1].min_stop_dis)
                        print "target in closed sector: closer to ii + 1"
                ii += 1
            # --- Emergency Avoid, Add in PF --- #
            if math.fabs(avoid_vector[0]) > 0 or math.fabs(avoid_vector[1]) > 0:  # something is close enough to avoid
                print "avoidvector", avoid_vector
                # change avoid vector to unit vector
                unit_avoid = [avoid_vector[0] / math.sqrt(avoid_vector[0]**2 + avoid_vector[1]**2),
                              avoid_vector[1] / math.sqrt(avoid_vector[0]**2 + avoid_vector[1]**2)]
                print 'unit avoid vector x y', unit_avoid
                print "x_y", x_y
                # add the void vector and open sector vector
                open_and_avoid = [unit_avoid[0] + x_y[0], unit_avoid[1] + x_y[1]]
                print "open_and_avoid", open_and_avoid
                print "math.sqrt(open_and_avoid[0]**2 + open_and_avoid[1]**2)", math.sqrt(open_and_avoid[0]**2 + open_and_avoid[1]**2)
                if sum(open_and_avoid) == 0: # added incase the avoid vector cancels out the cmd vector
                    x_cmd = 0
                    y_cmd = 0
                else:
                    x_cmd = open_and_avoid[0] / math.sqrt(open_and_avoid[0]**2 + open_and_avoid[1]**2)
                    y_cmd = open_and_avoid[1] / math.sqrt(open_and_avoid[0]**2 + open_and_avoid[1]**2)
                print "open and avoid", open_and_avoid
            else:
                x_cmd = max_speed*x_y[0]  # x_cmd = unit_x_y[0] / math.sqrt(pow(unit_x_y[0], 2) + pow(unit_x_y[1], 2))
                y_cmd = max_speed*x_y[1]  # y_cmd = unit_x_y[1] / math.sqrt(pow(unit_x_y[0], 2) + pow(unit_x_y[1], 2))

            print "command angle ", norm_2pi(math.atan2(y_cmd, x_cmd))


        else:  # use pf: no open sectors of ues_pf flag true (in pf radius of wp)
            global add_past
            if dmp_dis != 1 and add_past:
                target_angle_global = add_past_choice(target_angle_global, past_choices)
            pub_marker(target_angle_global - yaw, t_m, 'past_mod_target', [1, 1, .5, 0])
            t_x_global = t_m * math.cos(target_angle_global)
            t_y_global = t_m * math.sin(target_angle_global)
            t_x_local = math.cos(-yaw) * t_x_global - math.sin(-yaw) * t_y_global
            t_y_local = math.sin(-yaw) * t_x_global + math.cos(-yaw) * t_y_global
            # get componets of pf vector
            pf_x = pf[1] * math.cos(pf[0])
            pf_y = pf[1] * math.sin(pf[0])
            # add pf and target to get new_target vector
            x_cmd = pf_x + t_x_local
            y_cmd = pf_y + t_y_local
            print "t,", t_m

    else: # pos hold, avoid with pf
        pf_x = pf[1] * math.cos(pf[0])
        pf_y = pf[1] * math.sin(pf[0])
        x_cmd = pf_x + pos_x_cmd_body
        y_cmd = pf_y + pos_y_cmd_body

    # add avoid other quads
    if zero_vel_flag:
        cmd_angle = math.atan2(y_cmd,x_cmd)
        cmd_mag =  0 #math.sqrt(x_cmd**2+y_cmd**2)
        x_cmd = cmd_mag/2*math.cos(cmd_angle)
        y_cmd = cmd_mag/2*math.sin(cmd_angle)

    # local velocity for step check
    vel_x_local = math.cos(-yaw) * vel_x - math.sin(-yaw) * vel_y
    vel_y_local = math.sin(-yaw) * vel_x + math.cos(-yaw) * vel_y

    # --- filter cmd mag --- #
    global filter_cmd
    print "mag", math.sqrt(x_cmd**2+ y_cmd**2), 'max speed', max_speed, 't_m'

    cmd_angle = math.atan2(y_cmd, x_cmd)
    x_cmd, y_cmd = filter_cmd.add(x_cmd, y_cmd)  # change_speed(x_cmd, y_cmd, vel_x_local, vel_y_local, max_speed, msg.angular.z))

    if not pot_field_only:
        cmd_mag = math.sqrt(x_cmd ** 2 + y_cmd ** 2)
        if cmd_mag > t_m:
            cmd_mag = t_m
            x_cmd = cmd_mag*math.cos(cmd_angle)
            y_cmd = cmd_mag*math.sin(cmd_angle)
    else:
        cmd_mag = math.sqrt(x_cmd ** 2 + y_cmd ** 2)
        if cmd_mag > t_m:
            x_cmd = t_m*math.cos(cmd_angle)
            y_cmd = t_m*math.sin(cmd_angle)

    # # --- Step check --- #
    # step = 1  # was 0.75 for double bat frame
    # if abs(vel_x_local - x_cmd) > step:
    #     print "x reduced"
    #     if vel_x_local > x_cmd:
    #         x_cmd = vel_x_local - step
    #     if vel_x_local < x_cmd:
    #         x_cmd = vel_x_local + step
    #
    # if abs(vel_y_local - y_cmd) > step:
    #     print "y reduced"
    #     if vel_y_local > y_cmd:
    #         y_cmd = vel_y_local - step
    #     if vel_y_local < y_cmd:
    #         y_cmd = vel_y_local + step

    # --- Save past choice --- #
    if hold_flag == 0 and vel_flag == False :
        x_global = math.cos(yaw) * x_cmd - math.sin(yaw) * y_cmd
        y_global = math.sin(yaw) * x_cmd + math.cos(yaw) * y_cmd

        if pot_field_only:
            num_of_past = 90
        else:
            num_of_past = 90
        if gg > num_of_past - 1:
            gg = 0
        choice = xy(x_global, y_global)
        if len(past_choices) <= num_of_past:
            past_choices += [choice]
        else:
            past_choice_full_flag = True
            past_choices[gg] = choice
            gg += 1
    else:
        past_choices = []
        gg = 0
        past_choice_full_flag = False




    if mode == 1 or mode == 2: # Sim and DJI
        msg.linear.x = x_cmd
        msg.linear.y = y_cmd
    else:
        msg.linear.x = -y_cmd
        msg.linear.y = x_cmd
        msg.angular.z *= -math.pi / 180.0

    msg.angular.x = 0
    msg.angular.y = 0
    #msg.angular.z = 0



    #  Publish markers for visualization
    pub_marker(math.atan2(y_cmd, x_cmd),math.sqrt(x_cmd**2 + y_cmd**2), 'cmd', [1, 0, 1, 0]) # math.sqrt(pow(msg.linear.y, 2) + pow(msg.linear.x, 2))
    pub_marker(pf[0], pf[1], 'pure_pf', [1, 1, 1, 0])
    pub_marker(math.atan2(vel_y_local,vel_x_local), math.sqrt(vel_x_local**2 + vel_y_local**2), 'real_vel', [1,0,.5, 0])
    # publish cmd velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)    
    pub.publish(msg)
    pub_error = rospy.Publisher('cmd_vel_error', Twist, queue_size=1)
    pub_error.publish(msg_error)
    print "TIME", rospy.get_time(), '--------------------------------'
    print "\n\n"


def change_speed(cmd_x,cmd_y, real_x,real_y, cmd_speed, z_w):
    global angle_increment
    global seg
    global ranges
    real_angle = math.atan2(real_y, real_x)
    for sector in open_sector:
        if between(real_angle,sector.start,sector.stop):
            return math.sqrt(cmd_x**2 + cmd_y**2)
    return 1

def pub_waypoint(lat,lon, h):
    idx = 0
    topic = "goal_wp"
    global mode, bag
    if mode == 1:
        topic = topic + '_sim'
    elif bag == 1:
        topic = topic + '_bag'
    publisher = rospy.Publisher(topic, Vector3Stamped, queue_size=1)
    vector = Vector3Stamped()
    header = Header()
    # creating header
    header.seq = idx
    ftime_now = rospy.get_time()
    header.stamp = genpy.rostime.Time(int(ftime_now) // 1, int((ftime_now % 1.0) * 1000000000))
    header.frame_id = "laser"
    vector.header = header
    vector3 = Vector3()
    vector3.x = lat
    vector3.y = lon
    vector3.z = h
    vector.vector = vector3
    publisher.publish(vector)

def pub_marker(angle, mag, topic, color, frame='laser'):
    #print "PRINT ", topic
    global mode, bag
    if mode == 1:
        topic = topic + '_sim'
    elif bag == 1:
        topic = topic + '_bag'
    publisher = rospy.Publisher(topic, Marker, queue_size = 1)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
    # type(pose) = geometry_msgs.msg.Pose
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = mag
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

        # Publish the MarkerArray
    publisher.publish(marker)

if __name__ == '__main__':

    global mode
    rospy.init_node('listener', anonymous=True)

    if mode == 1:
        print "simulation mode"
        rospy.Subscriber("ground_truth_to_tf/pose", PoseStamped, main_cmd, queue_size=1)
        #rospy.Subscriber("new_scan_sim", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("seg_scan_sim", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("fix_velocity", Vector3Stamped, set_global_vel, queue_size=1)
        rospy.Subscriber("waypoint", Twist, set_waypoint, queue_size = 1)
        rospy.Subscriber("fix", NavSatFix, set_lat_lon, queue_size = 1)
        rospy.Subscriber("sonars", LaserScan, read_sonar, queue_size = 1)
        rospy.Subscriber('agent_mps', PoseStamped, callback_avoid_agents, queue_size=1)
    elif mode == 2:
        print "DJI mode"
        rospy.Subscriber("dji_sdk/attitude_quaternion", dji_sdk.msg.AttitudeQuaternion, main_cmd, queue_size=1)
        rospy.Subscriber("dji_sdk/global_position", dji_sdk.msg.GlobalPosition, set_lat_lon, queue_size=1)
        rospy.Subscriber("seg_scan", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("ext_height", Range, distSensor_callback, queue_size=1)
        rospy.Subscriber("dji_sdk/velocity", dji_sdk.msg.Velocity, set_global_vel, queue_size=1)
        rospy.Subscriber("CA_waypoint", dji_sdk.msg.Waypoint, set_waypoint, queue_size=1)
        rospy.Subscriber("waypoint_list", dji_sdk.msg.MissionWaypointTask, set_t_m, queue_size=1)
        rospy.Subscriber("ultrasonic_scan", LaserScan, read_sonar,queue_size=1)

    elif mode == 3:
        print "MAVROS mode"
        rospy.Subscriber("mavros/imu/data", Imu, main_cmd, queue_size=1)
        rospy.Subscriber("mavros/global_position/global", NavSatFix,set_lat_lon, queue_size=1)
        rospy.Subscriber("seg_scan", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("mavros/distance_sensor/lidarlite_pub", Range, distSensor_callback, queue_size=1)
        rospy.Subscriber("mavros/local_position/velocity", TwistStamped, set_global_vel, queue_size=1)
        rospy.Subscriber("agent_mps_data", enif_iuc.msg.AgentMPS, callback_avoid_agents, queue_size=1)
        rospy.Subscriber("CA_waypoint", enif_iuc.msg.Waypoint, set_waypoint, queue_size=1)
        rospy.Subscriber("waypoint_list", enif_iuc.msg.WaypointTask, set_t_m, queue_size=1)
        #rospy.Subscriber("agent_source_data", enif_iuc.msg.AgentSource, source_sub, queue_size=1)
        rospy.Subscriber("ultrasonic_scan", LaserScan, read_sonar,queue_size=1)
        rospy.Subscriber("agentState", UInt8, state_sub, queue_size=1)

    
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        if mavState<2:
            run_open_sim = rospy.get_param("/runAlg")
            
        r.sleep()
        if run_open_sim == 'info':
            print "[CA] skipping main_cmd and other subs runAlg=", run_open_sim
        else:
            print "running open_sim runAlg=", run_open_sim            
            

    rospy.spin()

