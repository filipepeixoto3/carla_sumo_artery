#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the carla simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import logging
import numpy as np
import carla  # pylint: disable=import-error
import math
import zmq
import json
import threading

from .constants import INVALID_ACTOR_ID, SPAWN_OFFSET_Z

from sumo_integration.bridge_helper import BridgeHelper

# ==================================================================================================
# -- carla simulation ------------------------------------------------------------------------------
# ==================================================================================================


class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, host, port, sumo_cfg, step_length, zmq_port=5555):
        self.client = carla.Client(host, port)
        self.client.set_timeout(30.0)
        self.host = host
        self.start_time = 0
        self.sumo_cfg = sumo_cfg

        self.world = self.client.get_world()
        
        self.blueprint_library = self.world.get_blueprint_library()
        self.step_length = step_length

        # The following sets contain updated information for the current frame.
        self._active_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()


        # Set traffic lights.
        self._tls = {}  # {landmark_id: traffic_ligth_actor}

        tmp_map = self.world.get_map()
        for landmark in tmp_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_ligth = self.world.get_traffic_light(landmark)
                if traffic_ligth is not None:
                    self._tls[landmark.id] = traffic_ligth
                else:
                    logging.warning('Landmark %s is not linked to any traffic light', landmark.id)
        # sensor data collection
        self.sensor_list = []
        self.send_receive=0
        self.uss_msg = dict()
        
        # ZMQ stuff
        self.zmq_port = zmq_port
        self.start_zmq_server()
        self.sensor_router_socket_list = []

        # Added by Marcelo
        self.sumo2carla_ids = {}
        

    def set_start_time(self,start_time):
        self.start_time = start_time 


    def start_zmq_server(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{self.host}:{self.zmq_port}")

        # Thread that forwards messages to the OMNeT++ side
        self.thread = threading.Thread(target=self.router)
        self.thread.start()

    def router(self):
        router_socket = self.context.socket(zmq.SUB)
        router_socket.bind("inproc://internalstuff")
        router_socket.subscribe("")

        poller = zmq.Poller()
        poller.register(router_socket, zmq.POLLIN)

        try:
            while not self.stop_event.is_set():
                try:
                    socks = dict(poller.poll(100))  # timeout 100 ms
                except zmq.error.ContextTerminated:
                    break

                if router_socket in socks and socks[router_socket] == zmq.POLLIN:
                    frames = router_socket.recv_multipart()
                    self.socket.send_multipart([frames[0], frames[1]])
        finally:
            router_socket.close()

    def get_actor(self, actor_id):
        """
        Accessor for carla actor.
        """
        return self.world.get_actor(actor_id)

    # This is a workaround to fix synchronization issues when other carla clients remove an actor in
    # carla without waiting for tick (e.g., running sumo co-simulation and manual control at the
    # same time)
    def get_actor_light_state(self, actor_id):
        """
        Accessor for carla actor light state.

        If the actor is not alive, returns None.
        """
        try:
            actor = self.get_actor(actor_id)
            return actor.get_light_state()
        except RuntimeError:
            return None

    @property
    def traffic_light_ids(self):
        return set(self._tls.keys())

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        if landmark_id not in self._tls:
            return None
        return self._tls[landmark_id].state

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(True)
                # We set the traffic light to 'green' because 'off' state sets the traffic light to
                # 'red'.
                actor.set_state(carla.TrafficLightState.Green)


    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new actor.
        ...
        """
        [...]
        if "vehicle" in blueprint.id:
            car = self.world.get_actor(response.actor_id)
            car_length = car.bounding_box.extent.x
            car_width = car.bounding_box.extent.y

            sensor_bp = self.blueprint_library.find("sensor.other.obstacle")
            sensor_bp.set_attribute("distance", "20")
            sensor_bp.set_attribute("hit_radius", "0.5")
            sensor_bp.set_attribute("only_dynamics", "false")
            sensor_bp.set_attribute("debug_linetrace", "true")
            sensor_bp.set_attribute("sensor_tick", "0.05")

            sensor_location = carla.Location(x=car_length, y=0, z=0.5)
            sensor_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)

            sensor_batch = []
            sensor_batch.append(carla.command.SpawnActor(sensor_bp, sensor_transform, response.actor_id))

            # Apply batch
            size = len(self.sensor_list)
            response_list = self.client.apply_batch_sync(sensor_batch, False)
            for idx, resp in enumerate(response_list):
                idx_ = idx + size
                if resp.error:
                    logging.error("Spawn sensor %s failed. %s", idx_, resp.error)
                else:
                    router_socket = self.context.socket(zmq.PUB)
                    router_socket.connect("inproc://internalstuff")
                    self.sensor_router_socket_list.append(router_socket)
                    sensor = self.world.get_actor(resp.actor_id)
                    self.sensor_list.append(sensor)

                    self.sensor_list[idx_].listen(
                        lambda sensor_msrmnt, idx_=idx_: self.sensor_callback(
                            sensor_msrmnt,
                            self.sensor_list[idx_].parent.id,
                            self.sensor_list[idx_].id,
                            self.sensor_router_socket_list[idx_],
                        )
                    )
    
    def sensor_callback(self, sensor_msrmnt, vehicle_id, sensor_id, zmq_socket):
        obst_msg = dict()
        obst_msg["message_type"] = "OBSTACLE_DATA"
        obst_msg["vehicle_id"] = int(list(self.sumo2carla_ids.keys())[list(self.sumo2carla_ids.values()).index(vehicle_id)])
        obst_msg["sensor_id"] = sensor_id
        obst_msg["timestamp"] = sensor_msrmnt.timestamp-self.start_time
        obst_msg["distance"] = sensor_msrmnt.distance

        print("    sensor", sensor_id, "@ car ", obst_msg["vehicle_id"], "distance =", sensor_msrmnt.distance, "timestamp =", sensor_msrmnt.timestamp-self.start_time)

        # Publish data via ZMQ
        if vehicle_id in self.sumo2carla_ids.values():
            carla2sumo_id = str(
                list(self.sumo2carla_ids.keys())[list(self.sumo2carla_ids.values()).index(vehicle_id)]
            )
        else:
            return

        topic = carla2sumo_id + ".obst" + str(sensor_id)
        zmq_socket.send_multipart([topic.encode("utf-8"), json.dumps(obst_msg).encode("utf-8")])

    def destroy_actor(self, actor_id):
        """
        Destroys the given actor.
        """
        actor = self.world.get_actor(actor_id)
        # Destroy all sensors attached to this actor
        cmd_batch = []
        for idx, sensor in enumerate(self.sensor_list):
            if sensor.parent.id==actor_id:
                self.sensor_list[idx].stop()
                self.sensor_router_socket_list[idx].close()
                cmd_batch.append(carla.command.DestroyActor(sensor.id))
        if cmd_batch:
            self.client.apply_batch_sync(cmd_batch, False)

        if actor is not None:
            return actor.destroy()
        return False

    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        """
        Updates vehicle state.
            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param lights: new vehicle light state.
            :return: True if successfully updated. Otherwise, False.
        """
        vehicle = self.world.get_actor(vehicle_id)
        if vehicle is None:
            return False

        vehicle.set_transform(transform)
        if lights is not None:
            vehicle.set_light_state(carla.VehicleLightState(lights))
        return True

    def animate_person(self, person, sumo_person, cur_stage, prev_stage, seat_params):
            
            # Sitting/driving vehicle pose
            if (cur_stage.description=="driving" and prev_stage.description=="walking"):

                # Extract information about vehicle the person is riding on.
                #frontSeatPos, seatingWidth = seat_params
                frontSeatPos = float(seat_params[0][1])
                seatingWidth = float(seat_params[1][1])
                height = seat_params[2]
                # use default values if empty
                if (frontSeatPos==0): frontSeatPos=2
                if (seatingWidth==0): seatingWidth=1.3
                if (height==0): height=1.5
                
                #print("driving on vtype=", cur_stage.vType, " height=", height)

                bones = person.get_bones()
                new_pose = []
                for bone in bones.bone_transforms:
                    # hands on the steering wheel
                    if bone.name == "crl_foreArm__L":
                        bone.relative.rotation.pitch -= 90
                        new_pose.append((bone.name, bone.relative))
                    elif bone.name == "crl_foreArm__R":
                        bone.relative.rotation.pitch -= 90
                        new_pose.append((bone.name, bone.relative))
                    # legs forwards
                    elif bone.name == "crl_thigh__R":
                        bone.relative.rotation.roll -= 90
                        new_pose.append((bone.name, bone.relative))
                    elif bone.name == "crl_thigh__L":
                        bone.relative.rotation.roll -= 90
                        new_pose.append((bone.name, bone.relative))
                    # knees bent slightly downwards
                    elif bone.name == "crl_leg__R":
                        bone.relative.rotation.roll -= 10
                        new_pose.append((bone.name, bone.relative))
                    elif bone.name == "crl_leg__L":
                        bone.relative.rotation.roll -= 10
                        new_pose.append((bone.name, bone.relative))
                    # offset to put person in the driver's steat     
                    elif bone.name == "crl_root":
                        bone.relative.location.x += seatingWidth/4
                        bone.relative.location.y -= frontSeatPos
                        bone.relative.location.z -= 0.5
                        new_pose.append((bone.name, bone.relative))

                control = carla.WalkerBoneControlIn()
                control.bone_transforms = new_pose
                person.set_bones(control)
                person.blend_pose(1.0)
            elif (cur_stage.description=="walking"):
                control = carla.WalkerControl(
                            speed=sumo_person.speed,
                            direction=carla.Vector3D(x=1.0, y=0.0, z=0.0),
                            jump=False)
                person.apply_control(control)
                person.blend_pose(0.0)

    def animate_actor(self, actor, current_yaw, previous_yaw, speed):
        diff=current_yaw-previous_yaw
        factor=1/(self.step_length*5)
        
        # make wheels turns. they don't spin. spinning requires CARLA phyisics which we do not desire.
        actor.set_wheel_steer_direction(carla.VehicleWheelLocation.FL_Wheel, diff*factor)    # diff*factor
        actor.set_wheel_steer_direction(carla.VehicleWheelLocation.FR_Wheel, diff*factor)    # diff*factor

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param landmark_id: id of the landmark to be updated.
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        if not landmark_id in self._tls:
            # logging.warning('Landmark %s not found in carla', landmark_id)
            return False

        traffic_light = self._tls[landmark_id]
        traffic_light.set_state(state)
        return True

    def tick(self):
        """
        Tick to carla simulation.
        """
        self.world.tick()

        # Update data structures for the current frame.
        current_actors = set(
            [vehicle.id for vehicle in self.world.get_actors().filter('vehicle.*')])
        self.spawned_actors = current_actors.difference(self._active_actors)
        self.destroyed_actors = self._active_actors.difference(current_actors)
        self._active_actors = current_actors

        # Same thing, but for persons.
        current_persons = set(
            [walker.id for walker in self.world.get_actors().filter('walker.*')])
        self.spawned_person = current_persons.difference(self._active_persons)
        self.destroyed_persons = self._active_persons.difference(current_persons)
        self._active_persons = current_persons

    def close(self):
        """
        Closes carla client.
        """
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(False)

        if hasattr(self, "stop_event"):
            self.stop_event.set()

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=1)
        try:
            if hasattr(self, "socket"):
                self.socket.close(0)
        except Exception:
            pass
        try:
            if hasattr(self, "context"):
                self.context.term()
        except Exception:
            pass
