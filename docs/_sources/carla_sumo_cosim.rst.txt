CARLA and SUMO Setup
====================

This section explains how to adapt the CARLA–SUMO co-simulation framework so it can interface with a network simulator through ZeroMQ.  
CARLA has supported native SUMO co-simulation since version 0.9.8, enabling synchronized actions and state between both simulators.  
Here, we extend this functionality to broadcast CARLA sensor data to an external network simulator.

All required integration files are located in:

``Path/To/CARLA/Co-Simulation/Sumo``

Make a copy of that directory and rename it to something like:

``sumo_omnetpp``  

This new directory will hold all modifications needed to connect CARLA, SUMO, and Artery.

carla_simulation.py
-------------------

ZeroMQ (ZMQ) is the central component that enables communication between CARLA and Artery.  
Begin by modifying:

``Path/To/CARLA/Co-Simulation/sumo_omnetpp/sumo_integration/carla_simulation.py``

Add the required import at the top of the file:

.. code-block:: python
   :linenos:
   :caption: Import ZeroMQ in carla_simulation.py

   import zmq


Constructor Updates
~~~~~~~~~~~~~~~~~~~

Modify the :meth:`CarlaSimulation.__init__` function so that it:

1. Accepts a ZMQ port (default 5555)  
2. Initializes the ZMQ server  
3. Creates lists for router sockets and CARLA sensors  
4. Stores a mapping between SUMO vehicle IDs and CARLA vehicle IDs  

.. code-block:: python
   :linenos:
   :caption: CarlaSimulation.__init__ modifications

   class CarlaSimulation(object):
       """
       CarlaSimulation is responsible for managing the CARLA simulation instance.
       """
       def __init__(self, host, port, step_length, zmq_port=5555):
           [...]
            self.host = host
            self.start_time = 0
           # ZMQ
           self.zmq_port = zmq_port
           self.start_zmq_server()
           self.sensor_router_socket_list = []
           self.sensor_list = []
           self.sumo2carla_ids = {}  # maps SUMO vehicle IDs to CARLA vehicle IDs
           [...]

Add a setter for the start timestamp:

.. code-block:: python
   :linenos:
   :caption: Set start time

    def set_start_time(self,start_time):
        self.start_time = start_time

ZMQ Server and Router Thread
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Next, define the ZMQ server and internal router.  
The server publishes sensor data externally, while the router listens to in-process channels and forwards messages.

.. code-block:: python
   :linenos:
   :caption: start_zmq_server and router methods

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

Sensor Integration
~~~~~~~~~~~~~~~~~~

Before adding sensors, note that **camera-based sensors are not suitable** for this architecture.  
Their frames must be processed externally, and transmitting raw images over ZMQ is inefficient.  
Instead, use lightweight numeric sensors such as:

* ``sensor.other.obstacle``  
* ``sensor.lidar.ray_cast``  
* ``sensor.other.radar``  

Obstacle Sensor Attachment
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each vehicle will receive one obstacle sensor upon creation.  
Append the following block to the end of :meth:`CarlaSimulation.spawn_actor`:

.. code-block:: python
   :linenos:
   :caption: Spawning an obstacle sensor with each vehicle

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

Sensor Callback
~~~~~~~~~~~~~~~~

This function receives sensor data, formats the message, and publishes it through ZMQ.

.. code-block:: python
   :linenos:
   :caption: sensor_callback method

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

Actor Destruction
~~~~~~~~~~~~~~~~~

.. code-block:: python
   :linenos:
   :caption: destroy_actor method

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

Close Method
~~~~~~~~~~~~

.. code-block:: python
   :linenos:
   :caption: close method to gracefully stop CARLA

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

run_synchronization.py
----------------------

Import ``carla`` at the top of the script:

.. code-block:: python
   :linenos:
   :caption: import carla in run_synchronization.py

   import carla

Update the constructor to pass the SUMO–CARLA ID map:

.. code-block:: python
   :linenos:
   :caption: SimulationSynchronization.__init__

   class SimulationSynchronization(object):
       """
       Handles the synchronization between SUMO and CARLA simulations.
       """
       def __init__(self, sumo_simulation, carla_simulation, tls_manager="none",
                    sync_vehicle_color=False, sync_vehicle_lights=False):
           [...]
           self.carla.sumo2carla_ids = self.sumo2carla_ids

Initialize the CARLA start time:

.. code-block:: python
   :linenos:
   :caption: Set start time in synchronization_loop

    class SimulationSynchronization(object):
        
        def synchronization_loop(args):
            """
            Entry point for sumo-carla co-simulation.
            """
            [...]
            try:
                snapshot = carla_simulation.world.get_snapshot()
                start_time = snapshot.timestamp.elapsed_seconds
                carla_simulation.set_start_time(start_time)
                [...]
            [...]

Add a command-line option for the ZMQ port:

.. code-block:: python
   :linenos:
   :caption: Command-line argument for ZMQ port

   if __name__ == "__main__":
       [...]
       argparser.add_argument(
           "--zmq-port",
           metavar="P",
           default=5555,
           type=int,
           help="Port used for the local ZMQ socket (default: 5555)",
       )
       [...]

Summary
-------

In this section, you:

* Extended the default CARLA–SUMO co-simulation  
* Added ZeroMQ support for external communication  
* Attached obstacle sensors to CARLA vehicles  
* Implemented a callback to publish sensor data  
* Updated the synchronization script to propagate vehicle IDs  
* Added support for configurable ZMQ ports  

With this setup, CARLA and SUMO can now send obstacle measurements to a network simulator such as Artery.

Continue
~~~~~~~~

Proceed to the next section:

* :doc:`artery_setup` – configuring Artery to receive CARLA sensor data
