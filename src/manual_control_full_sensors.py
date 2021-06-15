#!/usr/bin/env python 
# Javier Araluce

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

# from examples.manual_control import (CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor)

from openface_utils.carla_utils import CameraManager, HUD, KeyboardControl, CameraManagerRGB, CameraManagerDepth, CameraManagerSemantic, World, RepeatTimer, CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor

import os
import argparse
import logging
import time
import pygame
import math


import rospy
from carla_utils_msgs.msg import DriveMode, Float64Stamped
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from openface_utils import geometric_functions
import numpy as np

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class WorldSR(World):

    restarted = False

    def restart(self):

        self.pub = rospy.Publisher('carla/hero/drive_mode', DriveMode, queue_size = 10)
        self.pub_velocity = rospy.Publisher('carla/hero/velocity', Float64, queue_size = 10)
        self.pub_ttc = rospy.Publisher('carla/hero/ttc', Float64Stamped, queue_size = 10)
        self.pub_line_error = rospy.Publisher('carla/hero/line_error', Float64Stamped, queue_size = 10)
        self.pub_steer_cmd = rospy.Publisher('carla/hero/steer_cmd', CarlaEgoVehicleControl, queue_size = 10)
        self.pub_end_experiment = rospy.Publisher('t4ac/transition_experiment/end_experiment', Bool, queue_size = 10)

        rospy.Subscriber("/t4ac/v2u/gaze_focalization/gaze_focalization", PointStamped, self.callback_gaze)

        self.gaze = PointStamped()

        if self.restarted:
            return
        self.restarted = True

        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        # Keep same camera config if the camera manager exists.
        # cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        # cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        cam_index = self.camera_manager_rgb.index if self.camera_manager_rgb is not None else 0
        cam_pos_index = self.camera_manager_rgb._transform_index if self.camera_manager_rgb is not None else 0

        # Get the ego vehicle
        while self.player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == "hero":
                    print("Ego vehicle found")
                    self.player = vehicle
                    break
        
        self.player_name = self.player.type_id

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        # self.camera_manager.transform_index = cam_pos_index
        # self.camera_manager.set_sensor(cam_index, notify=False)


        if self.rgb_flag:
            self.camera_manager_rgb = CameraManagerRGB(self.player, self.hud, self._gamma, self.args_width, self.args_height)
            self.camera_manager_rgb._transform_index = cam_pos_index
            self.camera_manager_rgb.set_sensor(cam_index, notify=False)
        if self.depth_flag:
            self.camera_manager_depth = CameraManagerDepth(self.player, self.hud, self._gamma, self.args_width, self.args_height)
            self.camera_manager_depth._transform_index = cam_pos_index
            self.camera_manager_depth.set_sensor(cam_index, notify=False)
        if self.semantic_flag:
            self.camera_manager_semantic = CameraManagerSemantic(self.player, self.hud, self._gamma, self.args_width, self.args_height)
            self.camera_manager_semantic._transform_index = cam_pos_index
            self.camera_manager_semantic.set_sensor(cam_index, notify=False)    

        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def render(self, display, camera_rendered):
        if camera_rendered == 1   and self.rgb_flag: self.camera_manager_rgb.render(display)
        elif camera_rendered == 2 and self.semantic_flag: self.camera_manager_semantic.render(display)
        # elif camera_rendered == 3 and self.lidar_flag: self.camera_manager_lidar.render(display)
        elif camera_rendered == 4 and self.depth_flag: self.camera_manager_depth.render(display)
        if camera_rendered != self.previous_rendered:
            self.hud.notification(self.sensors[camera_rendered])
            if self.sensor_flags[camera_rendered-1] == 0: self.hud.notification("Not rendered sensor. Relaunch agent and activate sensor through Python arguments")

        self.previous_rendered = camera_rendered #detects if rendered sensor has changed
        self.hud.render(display)

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False

        self.hud.tick(self, clock)
        return True

    def destroySensors(self):  
            self.camera_manager_rgb.sensor.destroy()
            self.camera_manager_rgb.sensor = None
            self.camera_manager_rgb._index = None

    def destroy(self):
        self.end_experiment_pub(True)
        actors = [
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]

    def drive_mode_pub(self, change_mode, autopilot):
        msg = DriveMode()
        msg.header.stamp = rospy.Time.now() 
        msg.transition = change_mode
        if autopilot:
            msg.autonomous = True
            msg.manual = False
        else:
            msg.autonomous = False
            msg.manual = True

        self.pub.publish(msg)

    def ttc_pub(self, ttc):
        msg = Float64Stamped()
        msg.header.stamp = rospy.Time.now() 
        msg.data = ttc
        self.pub_ttc.publish(msg)

    def line_error_pub (self, error):
        msg = Float64Stamped()
        msg.header.stamp = rospy.Time.now() 
        msg.data = error
        self.pub_line_error.publish(msg)    

    def callback_gaze(self, msg):
        self.gaze =  msg

    def steer_cmd_pub(self, steer_cmd, brake_cmd, thorttle_cmd):
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = rospy.Time.now() 
        msg.steer = steer_cmd
        msg.brake = brake_cmd
        msg.throttle = thorttle_cmd

        self.pub_steer_cmd.publish(msg)

    def end_experiment_pub(self, flag):
        msg = Bool()
        msg.data = flag
        self.pub_end_experiment.publish(msg)


# ==============================================================================
# -- change_mode() ---------------------------------------------------------------
# ==============================================================================

# def autonomous_to_manual_mode(localization, map):
#     # print('X: ', round(localization.x), 'Y: ', round(localization.y))
#     if map.name == 'Town01':
#         if ((round(localization.x) == 305) and (193 < round(localization.y) < 197)):
#             change = True
#         elif ((round(localization.x) == 142) and (193 < round(localization.y) < 197)):
#             change = True
#         elif ((round(localization.x) == 42) and (325 < round(localization.y) < 328)):
#             change = True
#         else:
#             change = False
#     elif map.name == 'Town04':
#         if ((-305 < round(localization.x) < (-301)) and (433 < round(localization.y) < 438)):
#             change = True
#         elif ((411 < round(localization.x) < 414) and (-163 < round(localization.y) < -160)):
#             change = True
#         elif ((115 < round(localization.x) < 119) and ((-394) < round(localization.y) < (-388))):
#             change = True
#         else:
#             change = False
#     elif map.name == 'Town03':
#         if ((98 < round(localization.x) < 100) and (3 < round(localization.y) < 10)):
#             change = True
#         else:
#             change = False
#     else:
#         # print('mal')
#         change = False

          
#     return change



def autonomous_to_manual_mode(world, localization, map, transition_time, flag_change):
    """
    Function that change the driving mode when the car is at 20 metres above the other
    """
    vehicles = world.world.get_actors().filter('vehicle.*')
    distance_to_vehicle = 20
    for vehicle in vehicles:
        if (vehicle.id == 87) :
            object_location = vehicle.get_location()            
            euclidean_distance = math.sqrt(pow((localization.x - object_location.x), 2) + pow((localization.y - object_location.y), 2))
            if (euclidean_distance < distance_to_vehicle) and (flag_change == False):
                change = True
                flag_change = True
            else:
                change = False
          
    return change, flag_change   


def vehicle_to_bbox(linear_velocity, angular_velocity, transform, angle, dt):
    """
    Transform vehicle to a bbox, knowing velocity and position 
    """
    vehicle_width = 1.7
    vehicle_length = 4.4
    abs_vel = math.sqrt(pow(linear_velocity.x,2) + pow(linear_velocity.y,2))
    x_dt = transform.location.x + abs_vel * dt * math.cos(angle) # x centroid
    y_dt = transform.location.y + abs_vel * dt * math.sin(angle) # y centroid
    scale = vehicle_width * vehicle_length # scale is assumed to be constant for the ego-vehicle
    aspect_ratio = vehicle_width / vehicle_length # aspect_ratio is assumed to be constant for the ego-vehicle

    w = vehicle_width
    h = scale / w

    theta = angle + angular_velocity.z * dt * math.cos(angle) # Theta (orientation). 


    bbox = [x_dt, y_dt, w, h, theta]
    return bbox

def time_to_collision(world):
    """
    Time to collision between ego_vehicle and other dynamics objects 
    """

    ego_vehicle_linear_velocity = world.player.get_velocity()
    ego_vehicle_angular_velocity = world.player.get_angular_velocity() 
    ego_vehicle_transform = world.player.get_transform()
    vehicles = world.world.get_actors().filter('vehicle.*')
    ttc = []
    vehicle_id = []

    for vehicle in vehicles:
        if vehicle.id != world.player.id:
            object_velocity_linear = vehicle.get_velocity()
            object_velocity_angular = world.player.get_angular_velocity() 
            object_transform = vehicle.get_transform()


            for dt_10 in range(0, 30): # 3 seconds window
                if dt_10 == 0:
                    angle_ego = ego_vehicle_transform.rotation.yaw
                    angle_object = object_transform.rotation.yaw
                else:
                    angle_ego = previous_angle_ego
                    angle_object = previous_angle_object

                dt = dt_10 * 0.1
                ego_vehicle_bbox = vehicle_to_bbox(ego_vehicle_linear_velocity, ego_vehicle_angular_velocity, ego_vehicle_transform, angle_ego, dt)
                previous_angle_ego = ego_vehicle_bbox[4] # Save theta for next iteration
                object_bbox = vehicle_to_bbox(object_velocity_linear, object_velocity_angular, object_transform, angle_object, dt)
                previous_angle_object = object_bbox[4] # Save theta for next iteration

                area_iou = geometric_functions.iou(ego_vehicle_bbox, object_bbox) # Compute iou between both bbox

                if (area_iou > 0.0):
                    ttc.append(dt)
                    vehicle_id.append(vehicle.id)
                    break
                    
    if (len(ttc) > 0):
        min_ttc = min(ttc)
        world.ttc_pub(min_ttc)
        return min_ttc
    else:
        world.ttc_pub(1000)
        return 1000    
    
def line_displacement(world, current_position, initial_position):
    x_0 = initial_position.x
    y_0 = initial_position.y

    x_t = current_position.x
    y_t = current_position.y

    x_error = x_t - x_0
    y_error = y_t - y_0
    world.line_error_pub(y_error)

    # print(x_error, y_error)
    return y_error


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):

    pygame.init()
    pygame.font.init()
    world = None
    RED =   (255,   0,   0)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        traffic_manager = client.get_trafficmanager(int(8000))
        traffic_manager.global_percentage_speed_difference(-50)


        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)# | pygame.FULLSCREEN)

        hud = HUD(args.width, args.height)
        world = WorldSR(client.get_world(), hud, args)
        controller = KeyboardControl(world, args.autopilot)
        img_focus = pygame.image.load("images/logo_green2.png")
        img_focus = pygame.transform.scale(img_focus, (150, 150))
        img_unfocus = pygame.image.load("images/logo_red.png")
        img_unfocus = pygame.transform.scale(img_unfocus, (150, 150))
        x = args.width * 1 / 3; # x coordnate of image
        y = 0; # y coordinate of image
       

        town = world.map

        flag_change = False

        clock = pygame.time.Clock()
        while not rospy.core.is_shutdown():
            v = world.player.get_velocity()
            velocity = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
            

            
            ttc = time_to_collision(world) # Time to collision
            # print(ttc)

            current_position = world.player.get_transform().location
            waypoint = town.get_waypoint(world.player.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
            initial_position = waypoint.transform.location
            line_error = line_displacement(world, current_position, initial_position)

            hud.autopilot_enabled = controller._autopilot_enabled
            change_mode, flag_change = autonomous_to_manual_mode(world, current_position, town, args.transition_timer, flag_change)


            if (change_mode and controller.flag_timer == False):
                controller.flag_timer = True

                controller.begin_timer(world)
            
            clock.tick_busy_loop(60) # Maximun fps client
            if controller.parse_events(client, world, clock):
                return
            if not world.tick(clock):
                return
            # world.render(display)
            
            world.render(display, controller.camera_rendered) 

            # Draw gaze on screen 
            pygame.draw.circle(display, RED, [world.gaze.point.x + 1920, world.gaze.point.y], 10)

            if ((world.gaze.point.x > 0) and (world.gaze.point.x < 1920 ) 
                and (world.gaze.point.y > 0) and (world.gaze.point.y < 1080)):
                display.blit(img_focus, ( x,y)) # paint to screen
                controller.attention = True
                controller.flag_attention = True
            else:
                display.blit(img_unfocus, ( x,y)) # paint to screen
                controller.attention = False
                    
            pygame.display.flip()

            # Pub topics on ros for evaluation
            world.pub_velocity.publish(int(velocity))
            world.drive_mode_pub(controller.flag_timer, hud.autopilot_enabled)
            world.steer_cmd_pub(controller.steer_cmd, controller.brake_cmd, controller.thorttle_cmd)
            world.end_experiment_pub(False)

    finally:
        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()
 
        pygame.quit()
        


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():

    """
    main function
    """
    rospy.init_node('carla_manual_control_scenario', anonymous=True)

    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-t', '--transition_timer',
        metavar='T',
        default=3,
        type=int,
        help='transition_timer')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)5760x1080') 
    args = argparser.parse_args()

    args.rolename = 'hero'      # Needed for CARLA version
    args.filter = "vehicle.*"   # Needed for CARLA version
    args.gamma = 2.2   # Needed for CARLA version
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        print(error)
        logging.exception(error)


if __name__ == '__main__':

    main()
