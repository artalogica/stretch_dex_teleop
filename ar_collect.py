'''
This is our current implementation which consists of:
    1. set up ZMQ connection
    2. set up the initial position of controller on first iteration
    3. within the while True loop:
        a. read controller information from ZMQ
        b. generate new configuration
           call gripper_to_goal.update_goal to execute the new goal

We modified the original update_goal method to fit our new configuration format
'''

import controller_to_robot as c2r
import dex_teleop_parameters as dt
import numpy as np
import time
import zmq
import json
import argparse
import datetime 
import pyrealsense2 as rs
import cv2
import imageio
import os 
import csv 


'''
Parse the command line argument
1. The argument -i is the interval of responding the controller data
2. The argument -s is the speed of the robot, the choices include slow or fastest_stretch_2
'''
parser = argparse.ArgumentParser()
parser.add_argument('-i', '--interval', type=int, 
                    default=10, help='Interval in seconds')
parser.add_argument('-s', '--speed', choices=['slow', 'fastest_stretch_2'],
                    default='fastest_stretch_2', help='Speed option (choices: slow, fastest_stretch_2)')
args = parser.parse_args()

''' ZMQ coniguration'''
context = zmq.Context()
socket = context.socket(zmq.PULL)

''' Quest configuration'''
# Quest 3 IP on CMU-DEVICE
#socket.connect("tcp://172.26.187.122:12345")
socket.connect("tcp://172.26.187.122:23456")
# Quest 3 IP on Jensen's WIFI
# socket.connect("tcp://192.168.1.179:12345")

''' Robot configuration'''
robot_speed = args.speed
manipulate_on_ground = False
robot_allowed_to_move = True
using_stretch_2 = True


''' Camera configuration '''
save_video_color = True
save_video_depth = True
fps_color = 6
fps_depth_downsample_factor = 1 #6
resolution_color = [640, 480]
resolution_depth = [640, 480]
frameBuffer_limit_s = 10 # How many seconds of frames to keep in the buffer - probably something short to avoid memory usage issues
# Some image processing options.
apply_local_histogram_equalization = False
apply_global_histogram_equalization = False
frameBuffer_color = []
frameBuffer_depth = []


#get the serial number of camera
serial_no = None
rs_cameras = [{'name': device.get_info(rs.camera_info.name), 'serial_number': device.get_info(rs.camera_info.serial_number)}
    for device in rs.context().devices]

for info in rs_cameras:
    if 'D435' in info['name']:
        serial_no = info['serial_number']
        print(serial_no)
    if serial_no is None:
        print("Head D435 camera not found. Exiting.")
        sys.exit(1)

# Configure depth and color streams.
# cam 1: 3rd person view camera, cam 2: sideview camera
# if "topdown" in args.view:

#configure streaming to oculus 
cam_context = zmq.Context()
cam_sock = context.socket(zmq.PUB)

cam_sock.bind("tcp://172.26.189.111:12345") #claw meta 2's ip address 

pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device(serial_no)
config_1.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, fps_color) # note that the fps downsampling will be applied later
config_1.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, fps_color)
frame_width_color_1 = resolution_color[0] #resolution_color[1]
frame_height_color_1 = resolution_color[1] #resolution_color[0]
frame_width_depth_1 = resolution_color[0] #resolution_depth[1]
frame_height_depth_1 = resolution_color[1] #resolution_depth[0]
if pipeline_1 is not None:
    pipeline_1.start(config_1)

start_capture_time_s = time.time()
end_capture_time_color_s = start_capture_time_s
end_capture_time_depth_s = start_capture_time_s
frame_count_color = 0
frame_count_depth = 0
episode_count = 0
frame_count = 0 
frame_buffer_count = 0 
json_file_count = 0 
# print("Press Ctrl-C to exit...")
# try:
# set video writer for the color and depth video
writer_color_1 = None
writer_depth_1 = None

color_frame_1 = None
depth_frame_1 = None


'''Call the gripper to goal class for AR teleop logic with the robot configuration'''
gripper_to_goal = c2r.GripperToGoal(robot_speed,
    dt.get_starting_configuration(dt.get_lift_middle(manipulate_on_ground)),
    robot_allowed_to_move,
    using_stretch_2
)

'''
Set up the initial configuration of the update goal
This initial movement is just to demonstrate the script has launched, 
    the robot can move, and to move the arm to a good initial position.
The argument are set based on user experience

1. joint_lift is set to 0.6, which is similar to the position of human
2. the right and left trigger are set to 0, to make sure the initialization of gripper remain the same place
3. the right safety and left safety are set to 1 to enable the safety lock so robot is able to move to initial position
4. joint_arm_10 is set to 
5. q1, q2, q3, q4 are the quaternion form
6. right_thumbstick_x is the base rotation, it is set to 0 in the initial configuration
7. right_thumbstick_y is the base translation, it is set to 0 in the initial configuration
'''
base_rotation = 0
initial_configuration = {
    'joint_mobile_base_rotation': base_rotation,
    'joint_lift': 0.6,
    'right_trigger_status': 0, # range: 0~1
    'left_trigger_status': 0, # range: 0~1
    'right_safety': 1, # range: 0~1
    'left_safety': 1, # range: 0~1
    'joint_arm_l0': 0.25,
    'q1':-0.04,
    'q2':0.19,
    'q3':-0.08,
    'q4':0.97,
    'right_thumbstick_x':0,
    'right_thumbstick_y':0,
    'right_button_a': False,
    'right_button_b': False
}
gripper_to_goal.update_goal(initial_configuration)

# Sleep after reaching position to ensure stability
time.sleep(5)

response_cnt = 0
interval = args.interval

log_interval = interval * 3
 # Get the current date and time
current_datetime = datetime.datetime.now()
date_time_str = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
# Define the file path for the JSON file
task = input("Task: ")

json_file_path = f"data/{task}/{date_time_str}"

folder_path = f"data/{task}"
# Create the folder if it doesn't exist
os.makedirs(folder_path, exist_ok=True)


buffer_agent_state = { 
                'frame count': [], 
                'joint_mobile_base_rotation': [] ,
                'joint_lift': [],
                'right_trigger_status': [], # 0~1
                'left_trigger_status': [], # 0~1
                'right_safety': [], # 0~1
                'left_safety': [], # 0~1
                'joint_arm_l0': [],
                'q1': [],
                'q2': [],
                'q3': [],
                'q4': [],
                'right_thumbstick_x':[],
                'right_thumbstick_y':[],
                'right_button_a': [],
                'right_button_b': [], 
                'time': []
                #'color image': [], 
                #'depth image': []
                }

fieldnames = list(buffer_agent_state.keys())

"""prepare video capture paths"""

output_filepath_color = "data/" + date_time_str + "color.mp4"
output_filepath_depth = "data/" + date_time_str + "depth.mp4"
writer_color_1 = imageio.get_writer(output_filepath_color, fps=fps_color)
writer_depth_1 = imageio.get_writer(output_filepath_depth, fps=fps_color/fps_depth_downsample_factor)


 
while True:
    # Receive message from remote server
    message = socket.recv()
    try:
        data = json.loads(message.decode())
    except json.JSONDecodeError:
        print("Terminating the connection...")
        socket.close()
        cam_sock.close()
        context.term()
        cam_context.term()
        break

    if response_cnt % interval == 0:
        # Deserialize the JSON message
        right_controller = data["RightController"]
        left_controller = data["LeftController"]
        xyz = right_controller['RightLocalPosition']
        right_trigger_status = right_controller["RightIndexTrigger"]
        left_trigger_status = left_controller["LeftIndexTrigger"]
        right_safety = right_controller["RightHandTrigger"]
        left_safety = left_controller["LeftHandTrigger"]
        right_thumbstick_x, right_thumbstick_y = [float(x) for x in right_controller['RightThumbstickAxes'].split(',')]
        right_button_a = right_controller["RightA"]
        right_button_b = right_controller["RightB"]
        
        # Get the controller rotation in quaternion form
        controller_rotation = [float(x) for x in right_controller['RightLocalRotation'].split(',')]
        
        # Get the base rotation
        base_rotation_speed = 3.14/100 # magic number: the movement would be pi/100 everytime (radian based)
        if right_thumbstick_x > 0.5:
            base_rotation -= base_rotation_speed
        elif right_thumbstick_x < -0.5:
            base_rotation += base_rotation_speed

        # Get controller position and convert to joint lift and joint arm
        _, y_str, z_str = xyz.split(',')

        lift_pos = float(y_str)
        arm_ext_pos = float(z_str)
    
        mock_configuration = {
            'joint_mobile_base_rotation': base_rotation ,
            'joint_lift': lift_pos,
            'right_trigger_status': right_trigger_status, # 0~1
            'left_trigger_status': left_trigger_status, # 0~1
            'right_safety': right_safety, # 0~1
            'left_safety': left_safety, # 0~1
            'joint_arm_l0': arm_ext_pos,
            'q1': controller_rotation[0],
            'q2': controller_rotation[1],
            'q3': controller_rotation[2],
            'q4': controller_rotation[3],
            'right_thumbstick_x':right_thumbstick_x,
            'right_thumbstick_y':right_thumbstick_y,
            'right_button_a': right_button_a,
            'right_button_b': right_button_b
        }

        
    
        if response_cnt % log_interval == 0: 
            '''Grab the image from camera'''
            if pipeline_1 is not None:
                frames_1 = pipeline_1.wait_for_frames()
            if pipeline_1 is not None:
                depth_frame_1 = frames_1.get_depth_frame()
                color_frame_1 = frames_1.get_color_frame()
            if pipeline_1 is not None and (not depth_frame_1 or not color_frame_1):
                continue
            
            # Convert images to numpy arrays.
            if pipeline_1 is not None:
                depth_image_1 = np.asanyarray(depth_frame_1.get_data())
                color_image_1 = np.asanyarray(color_frame_1.get_data())

                img_serialized = cv2.imencode('.jpg', color_image_1)[1].tobytes()

                cam_sock.send(img_serialized)
            
            #write to img file labeling with the frame count 
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first).
            depth_image_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=-0.04, beta=255.0), cv2.COLORMAP_OCEAN)
            
            color_image_label = json_file_path + "color_" + str(frame_count) + ".jpg"
            depth_image_label = json_file_path + "depth_" + str(frame_count) + ".jpg"
            cv2.imwrite(color_image_label, color_image_1)
            cv2.imwrite(depth_image_label, depth_image_1)

            # frameBuffer_color.append(color_image_1.copy())
            # frameBuffer_depth.append(depth_image_1.copy())

            # writer_color_1.append_data(frameBuffer_color[-1][:, :, ::-1])
            # writer_depth_1.append_data(frameBuffer_depth[-1])

            current_datetime = datetime.datetime.now()
            curr_time = current_datetime.strftime("%Y-%m-%d %H:%M:%S")

            for key in mock_configuration: 
                buffer_agent_state[key].append(mock_configuration[key])

            buffer_agent_state['time'].append(curr_time)
            buffer_agent_state['frame count'].append(frame_count)

            frame_count = frame_count + 1 
            frame_buffer_count = frame_buffer_count + 1 
            if frame_buffer_count>=100: 
                #dump the json file 
                csv_file_base = json_file_path + "_data" + str(json_file_count) + ".csv"

                with open(csv_file_base, "w") as csvfile: 
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    # Write each row of data
                    for i in range(len(buffer_agent_state['time'])):
                        row = {key: buffer_agent_state[key][i] for key in fieldnames}
                        writer.writerow(row)

                json_file_count = json_file_count + 1
                frame_buffer_count = 0 
                buffer_agent_state = { 
                'info': task,
                'frame count': [], 
                'joint_mobile_base_rotation': [] ,
                'joint_lift': [],
                'right_trigger_status': [], # 0~1
                'left_trigger_status': [], # 0~1
                'right_safety': [], # 0~1
                'left_safety': [], # 0~1
                'joint_arm_l0': [],
                'q1': [],
                'q2': [],
                'q3': [],
                'q4': [],
                'right_thumbstick_x':[],
                'right_thumbstick_y':[],
                'right_button_a': [],
                'right_button_b': [], 
                'time': []
                #'color image': [], 
                #'depth image': []
                }
                print(f"Data has been saved to {csv_file_base}")

        # Send new state to update_goal
        gripper_to_goal.update_goal(mock_configuration)
    response_cnt+=1

# Write the data to a  csv_file 
print("writing to csv file...")
csv_file_base = json_file_path + "_data" + str(json_file_count) + ".csv"

with open(csv_file_base, "w") as csvfile: 
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    # Write each row of data
    for i in range(len(buffer_agent_state['time'])):
        row = {key: buffer_agent_state[key][i] for key in fieldnames}
        writer.writerow(row)

json_file_count = json_file_count + 1
frame_buffer_count = 0 
print(f"Data has been saved to {csv_file_base}")

pipeline_1.stop() 
if writer_color_1 is not None: 
    writer_color_1.close() 
if writer_depth_1 is not None: 
    writer_depth_1.close()

del gripper_to_goal