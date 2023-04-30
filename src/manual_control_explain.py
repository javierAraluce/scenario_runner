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

from explain_utils.carla_utils import CameraManager, HUD, KeyboardControl, \
    CameraManagerRGB, World, CollisionSensor, LaneInvasionSensor, GnssSensor, \
        IMUSensor

import os
import argparse
import logging
import time
import pygame
import math

from geometry_msgs.msg import PolygonStamped
import sensor_msgs.msg

import rospy
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
        
        
        
        self.action, self.reason = [], []
        self.action_label = ['Forward', 'Stop', 'Left', 'Right']
        self.reason_label = [
            ' follow traffic',
            'the road is clear', 
            'the traffic light is green', 
            'obstacle: car', 
            'obstacle: person/pedestrain', 
            'obstacle: rider', 
            'obstacle: others', 
            'the traffic light', 
            'the traffic sign', 
            'front car turning left', 
            'on the left-turn lane', 
            'traffic light allows', 
            'front car turning right', 
            'on the right-turn lane', 
            'traffic light allows', 
            "obstacles on the left lane", 
            "no lane on the left", 
            "solid line on the left", 
            "obstacles on the right lane", 
            "no lane on the right", 
            "solid line on the left"
]
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


        if self.rgb_flag:
            self.camera_manager_rgb = CameraManagerRGB(self.player, self.hud, self._gamma, self.args_width, self.args_height, 120, name = 'user')
            self.camera_manager_rgb._transform_index = cam_pos_index
            self.camera_manager_rgb.set_sensor(cam_index, notify=False)
            
            self.camera_manager_rgb_ros = CameraManagerRGB(self.player, self.hud, self._gamma, 1280, 720, 80, name = 'vehicle')
            # self.camera_manager_rgb_ros  = CameraManagerRGB(self.player, self.hud, self._gamma, self.args_width, self.args_height, 120, name = 'vehicle')
            self.camera_manager_rgb_ros._transform_index = 1
            self.camera_manager_rgb_ros.set_sensor(cam_index, notify=False)

        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        rospy.Subscriber("/t4ac/v2u/explainable_decision_making", 
                         PolygonStamped, 
                         self.model_callback)
        # input_image_topic = '/t4ac/perception/camera'
        rospy.Subscriber('/carla/hero/camera/rgb/vehicle/image_color', 
                         sensor_msgs.msg.Image, 
                         self.image_callback, 
                         queue_size=1)
        self.current_stamp = 0
        self.header = PolygonStamped().header

    def render(self, display, camera_rendered):
        if camera_rendered == 1   and self.rgb_flag: self.camera_manager_rgb.render(display)
        elif camera_rendered == 2   and self.rgb_flag: self.camera_manager_rgb_ros.render(display)
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
        # self.end_experiment_pub(True)
        actors = [
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]

    def model_callback(self, msg):
        self.header = msg.header
        self.action.clear()
        self.reason.clear()
        for point in msg.polygon.points:
            if point.x == 0.0:
                pass
            else:
                self.action.append(point.x)
            if point.y == 0.0:
                pass
            else:
                self.reason.append(point.y)
                
    def image_callback(self, img_msg):     
           
        self.current_stamp = img_msg.header.seq
        
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

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)# | pygame.FULLSCREEN)

        hud = HUD(args.width, args.height)
        world = WorldSR(client.get_world(), hud, args)
        controller = KeyboardControl(world, args.autopilot)
        
        arrow = pygame.image.load("images/arrow.png")
        arrow = pygame.transform.scale(arrow, (50, 50))
        
        forward_arrow = pygame.transform.rotate(arrow, 90)
        left_arrow = pygame.transform.rotate(arrow, 180)
        right_arrow = arrow
        break_arrow = pygame.transform.rotate(arrow, 270)
        
        x = args.width * 1 / 2 # x coordnate of image
        y = 0 # y coordinate of image

        x_velocity = args.width * 2 / 3 - 150
        y_velocity = args.height - 150
        
        x_action = args.width * 1 / 2 
        y_action = args.height * 1/7
        
        mixer = pygame.mixer
        mixer.init()
        sounds = {
            '0': mixer.Sound('sounds/0.mp3'),
            '1': mixer.Sound('sounds/1.mp3'),
            '2': mixer.Sound('sounds/2.mp3'),
            '3': mixer.Sound('sounds/3.mp3'),
            '4': mixer.Sound('sounds/4.mp3'),
            '5': mixer.Sound('sounds/5.mp3'),
            '6': mixer.Sound('sounds/6.mp3'),
            '7': mixer.Sound('sounds/7.mp3'),
            '8': mixer.Sound('sounds/8.mp3'),
            '9': mixer.Sound('sounds/8.mp3'),
            '10': mixer.Sound('sounds/10.mp3'),
            '11': mixer.Sound('sounds/11.mp3'),
            '12': mixer.Sound('sounds/12.mp3'),
            '13': mixer.Sound('sounds/13.mp3'),
            '14': mixer.Sound('sounds/14.mp3'),
            '15': mixer.Sound('sounds/15.mp3'),
            '16': mixer.Sound('sounds/16.mp3'),
            '17': mixer.Sound('sounds/17.mp3'),
            '18': mixer.Sound('sounds/18.mp3'),
            '19': mixer.Sound('sounds/19.mp3'),
            '20': mixer.Sound('sounds/20.mp3')
        }

        myFont = pygame.font.SysFont("Times New Roman", 100, bold=True)
        myFont_2 = pygame.font.SysFont("Times New Roman", 30, bold=True)

        clock = pygame.time.Clock()
        while not rospy.core.is_shutdown():
            v = world.player.get_velocity()
            velocity = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

            
            clock.tick_busy_loop(20) # Maximun fps client
            if controller.parse_events(client, world, clock):
                return
            if not world.tick(clock):
                return
            # world.render(display)
            
            world.render(display, controller.camera_rendered) 


            VelocityDisplay = myFont.render(str(int(velocity)), 1, (0,0,0))

            display.blit(VelocityDisplay, (x_velocity, y_velocity))  
            
            # # Display action 
            # # world.action = [1,3]
            # for action in  world.action:
            #     if action == 1: # Forward
            #         display.blit(forward_arrow, (x,y)) # paint to screen
            #     elif action == 2: # Stop
            #         display.blit(break_arrow, (x,y + 75)) # paint to screen
            #     elif action == 3: # Left
            #         display.blit(left_arrow, (x - 76 / 2,y + 76 / 2)) # paint to screen
            #     elif action == 4: # Right
            #         display.blit(right_arrow, (x + 76 / 2, y + 76 / 2)) # paint to screen
            #     else:
            #         pass
            
            # # world.reason = [4,6]
            # # Play explanations
            # str_header =  'Seq topic: ' + str(world.header.seq)
            # ReasonDisplay = myFont_2.render(str_header, 1, (0,0,0))
            # text_rect = ReasonDisplay.get_rect(center=(args.width/2, args.height/2))
            # display.blit(ReasonDisplay, (x_action, y_action + (30))) 
            
            # str_header =  'Seq current: ' + str(world.current_stamp)
            # ReasonDisplay = myFont_2.render(str_header, 1, (0,0,0))
            # text_rect = ReasonDisplay.get_rect(center=(args.width/2, args.height/2))
            # display.blit(ReasonDisplay, (x_action, y_action + (30 * 2))) 
            
            # for i, explanation in enumerate(world.reason):                
            #     if explanation != 0:
            #         explanation -=1
            #         # @TODO Play sound Problema con reproducir más de un sonido
                    
            #         # tada = mixer.Sound(sounds[str(explanation)])
            #         # channel = tada.play()
            #         reason_str = world.reason_label[int(explanation)]
            #         ReasonDisplay = myFont_2.render(reason_str, 1, (0,0,0))
            #         text_rect = ReasonDisplay.get_rect(center=(args.width/2, args.height/2))
            #         display.blit(ReasonDisplay, (x_action, y_action + 60 + (30 * i))) 
            #         # print(sounds[str(explanation)])
            #         # while channel.get_busy():
                    
                    
            #     else:
            #         pass

            pygame.display.flip()


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
        type=float,
        help='transition_timer')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720) 5760x1080') 
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
