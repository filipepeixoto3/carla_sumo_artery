Artery Setup
============

Follow the installation instructions provided at http://artery.v2x-research.eu/install/.

To integrate ZeroMQ (ZMQ) with Artery and allow it to receive CARLA sensor data, two main steps are required:
(1) enabling ZMQ support in the build system, and (2) developing a custom service that subscribes to the data published by CARLA and uses it inside Artery.

ZeroMQ Integration
------------------

First, modify the build configuration so that Artery can locate and link the ZMQ library.

Add the following lines to ``Path/To/Artery/src/artery/CMakeLists.txt``:

.. code-block:: c++
   :linenos:
   :caption: Path/To/Artery/src/artery/CMakeLists.txt modifications

   find_package(PkgConfig MODULE REQUIRED)
   pkg_check_modules(ZEROMQ REQUIRED libzmq)
   target_include_directories(core PUBLIC "${ZEROMQ_INCLUDES_DIRS}")
   target_link_libraries(core PUBLIC "${ZEROMQ_LIBRARIES}")

Once the project builds successfully with ZMQ support, you can implement the service that will receive obstacle information from CARLA.

Service Development
-------------------

To create a new Artery service, follow the procedure described at http://artery.v2x-research.eu/custom-simulation/.

In this integration, the implementation builds on the **ExampleService** pattern, but is implemented as **YourService**.  
The custom service subscribes to ZMQ messages sent by CARLA, parses the obstacle sensor data, computes the detected obstacle position, and passes the resulting information to other vehicles via Artery messages.

Create the following files in ``Path/To/Artery/src/artery/application``:

* ``YourService.cc``
* ``YourService.h``
* ``YourService.ned``
* ``YourServiceMessage.msg``

Add ``YourService.cc`` to the corresponding ``CMakeLists.txt`` file as indicated in the Artery documentation.

Message Definition
~~~~~~~~~~~~~~~~~~

The ``YourServiceMessage.msg`` file defines the information that will be exchanged between vehicles.

Create ``YourServiceMessage.msg`` in ``Path/To/Artery/src/artery/application/``:

.. code-block:: c++
    :linenos:
    :caption: YourServiceMessage.msg
 
    packet YourService
    {
        string messageType;
        double vehicleId;
        double sensorId;
        double initialTimestamp;
        double distance;
        double x;
        double y;
    };

Add the following line in ``Path/To/Artery/src/artery/CMakeLists.txt``, after the ``add_library`` section:

.. code-block:: c++
   :linenos:
   :caption: Register YourServiceMessage.msg in core target

    generate_opp_message(application/YourServiceMessage.msg TARGET core)

This line instructs OMNeT++ to generate the C++ message classes used by this service.

NED Service Declaration
~~~~~~~~~~~~~~~~~~~~~~~

A minimal NED definition for the service is:

.. code-block:: c++
   :linenos:
   :caption: YourService.ned

   package artery.application;

   simple YourService like ItsG5Service
   {
       parameters:
           string host = default(<MACHINE_RUNNING_CARLA_IP>);  // ZMQ host
           int port = default(5555);                 // ZMQ port
   }

JSON Support
~~~~~~~~~~~~

To parse the JSON messages sent from CARLA, download ``json.hpp`` from:

https://github.com/nlohmann/json/blob/develop/include/nlohmann/json.hpp

Place the file in:

* ``Path/To/Artery/src/artery/application/``

and include it in the CMake file similarly to how ``YourService.cc`` is added.

Include Libraries
~~~~~~~~~~~~~~~~~

In ``YourService.cc``, include the ZMQ, JSON and ``YourServiceMessage_m.h`` headers, as well as the vehicle controller:

.. code-block:: c++
    :linenos:
    :caption: Include libraries

    #include <zmq.hpp>
    #include "artery/application/json.hpp"
    #include "artery/application/YourServiceMessage_m.h"
    #include "artery/traci/VehicleController.h"

ZMQ Attributes
~~~~~~~~~~~~~~

Initialize the self-message pointer as ``nullptr`` and define the ZMQ socket, context and topic as private attributes of the service class:

.. code-block:: c++
   :linenos:
   :caption: ZMQ-related attributes in YourService

   namespace artery
   {
   class YourService : public ItsG5Service
   {
       //[...]
       private:
           //[...]
            std::string topic;
            omnetpp::cMessage* m_self_msg = nullptr;
            zmq::socket_t socket;
            zmq::context_t context;
   };
   }

ZMQ Initialization
~~~~~~~~~~~~~~~~~~

In :meth:`YourService.initialize`, create and configure the ZMQ subscriber.  
It will connect to the TCP endpoint exposed by the CARLA side and subscribe to the configured topic:

.. code-block:: c++
   :linenos:
   :caption: ZMQ socket initialization in YourService::initialize

   void YourService::initialize(){
       //[...]
       this->context = zmq::context_t{1};
       this->socket = zmq::socket_t{context, ZMQ_SUB};
       this->socket.setsockopt(ZMQ_RCVTIMEO, 100);  // set timeout to 4 seconds
       this->socket.setsockopt(ZMQ_SNDTIMEO, 100);  // set timeout to 4 seconds
       zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, topic.c_str(), topic.length());
       std::string host = par("host").stringValue();
       int port = par("port").intValue();
       std::string addr = "tcp://" + host + ":" + std::to_string(port);
       this->socket.connect(addr);
   }

Here:

* A ZMQ context and SUB socket are created.
* Receive and send timeouts are set (to avoid blocking indefinitely).
* The socket subscribes to the selected topic.
* The ``host`` and ``port`` parameters from NED are used to build the TCP address where the CARLA publisher is listening.

YourService Destructor
~~~~~~~~~~~~~~~~~~~~~~

At the end of the simulation, when the OMNeT++ modules are destroyed, the sockets and self-message must be properly cleaned up.  
This is accomplished in the following block:

.. code-block:: c++
   :linenos:
   :caption: YourService destructor

    YourService::~YourService()
    {
        EV_INFO << "Closing socket..." << endl;
        this->socket.close();
        EV_INFO << "Closing context..." << endl;
        this->context.close();
        if (m_self_msg != nullptr) { 
            cancelAndDelete(m_self_msg);
            m_self_msg = nullptr;
        }
    }

Receiving and Using ZMQ Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In :meth:`YourService.trigger`, before creating and sending the application packet, add the code that:

* receives messages from the ZMQ socket,
* parses the JSON content,
* computes the world coordinates of the detected obstacle,
* stores the result in a :cpp:type:`YourServiceMessage` instance, and
* sends the resulting message into the Artery communication stack.

.. code-block:: c++
    :linenos:
    :caption: Handling ZMQ data inside YourService::trigger

    void YourService::trigger(){
        //[...]
        auto& vehicleController = getFacilities().get_const<traci::VehicleController>();
        for (auto channel : mco.allChannels(example_its_aid)) {
            //[...]
            if (network) {
                //[...]
                this->socket.setsockopt(ZMQ_RCVTIMEO, 400);
                zmq::message_t reply{};
                int sndhwm;
                size_t sndhwm_size = sizeof(sndhwm);
                int rc;
                std::string uk = "null";
                std::list<int> sensors_with_input;

                do {
                    socket.recv(reply, zmq::recv_flags::dontwait);
                    rc = zmq_getsockopt(socket, ZMQ_RCVMORE, &sndhwm, &sndhwm_size);
                    if (reply.to_string().empty())
                        break;
                    if (reply.to_string()[0] == '{'){
                        json jsonResp = json::parse(reply.to_string());
                        std::string message_type = jsonResp["message_type"].get<std::string>();
                        double vehicle_id = jsonResp["vehicle_id"].get<double>();
                        double sensor_id = jsonResp["sensor_id"].get<double>();
                        double timestamp = jsonResp["timestamp"].get<double>();
                        double distance = jsonResp["distance"].get<double>();

                        EV << "    OBST-" << sensor_id << "<" << "@" << vehicle_id
                        << " (t=" << timestamp << ")  Distance=" << distance << "m\n";

                        double vehicle_length  = vehicleController.getLength().value();
                        double vehicle_width   = vehicleController.getWidth().value(); // unused but ok
                        double vehicle_angle   = vehicleController.getHeading().radian();

                        omnetpp::cFigure::Point cur_pos_center(
                            vehicleController.getPosition().x.value(),
                            vehicleController.getPosition().y.value());

                        double depth = distance;

                        // detection in vehicle-local coordinates (relative to center)
                        double det_x_v = vehicle_length / 2.0 + depth;
                        double det_y_v = 0.0;

                        double cosH = std::cos(vehicle_angle);
                        double sinH = std::sin(vehicle_angle);

                        double det_dx =  cosH * det_x_v - sinH * det_y_v;
                        double det_dy =  sinH * det_x_v + cosH * det_y_v;

                        omnetpp::cFigure::Point detection_coord =
                            cur_pos_center + omnetpp::cFigure::Point(det_dx, det_dy);

                        // sensor at front center
                        omnetpp::cFigure::Point sensor_pos =
                            cur_pos_center + omnetpp::cFigure::Point(vehicle_length / 2.0, 0.0);

                        omnetpp::cFigure::Point detection_pos(depth, 0.0);
                        omnetpp::cFigure::Point coord = sensor_pos + detection_pos;

                        auto* packet = new YourServiceMessage("YourService Packet");
                        packet->setMessageType(message_type.c_str());
                        packet->setVehicleId(vehicle_id);
                        packet->setSensorId(sensor_id);
                        packet->setInitialTimestamp(timestamp);
                        packet->setDistance(distance);
                        packet->setX(coord.x);
                        packet->setY(coord.y);
                        packet->setByteLength(42);

                        request(req, packet, network.get());
                    } else {
                        EV << "STRING=" << reply.to_string() << endl;
                        EV << "RCV_MORE=" << sndhwm << endl;
                    }
                } while (true);

                double vehicle_length  = vehicleController->getLength().value();
                double vehicle_width   = vehicleController->getWidth().value();
                double vehicle_angle   = mVehicleController->getHeading().radian();   // yaw [rad]

                // World position of vehicle center (use whatever Artery gives you here)
                omnetpp::cFigure::Point cur_pos_center(
                    mVehicleController->getPosition().x.value(),
                    mVehicleController->getPosition().y.value());
                double depth = distance;  // from your JSON

                // Detection in vehicle-local coordinates (relative to center)
                double det_x_v = vehicle_length / 2.0 + depth;  // forward
                double det_y_v = 0.0;                           // centered

                double cosH = std::cos(vehicle_angle);
                double sinH = std::sin(vehicle_angle);

                // Rotate from vehicle frame to world frame
                double det_dx =  cosH * det_x_v - sinH * det_y_v;
                double det_dy =  sinH * det_x_v + cosH * det_y_v;

                // Final world coordinates of the detection (method 1)
                omnetpp::cFigure::Point detection_coord =
                    cur_pos_center + omnetpp::cFigure::Point(det_dx, det_dy);

                vehicle_angle = mVehicleController->getHeading().radian();
                // Sensor at front center
                omnetpp::cFigure::Point sensor_pos =
                    cur_pos_center + omnetpp::cFigure::Point(vehicle_length / 2.0, 0.0);

                // Detection depth straight ahead of the sensor
                omnetpp::cFigure::Point detection_pos =
                    omnetpp::cFigure::Point(depth, 0.0);

                // World coordinate of detection (method 2)
                omnetpp::cFigure::Point coord = sensor_pos + detection_pos;

                YourServiceMessage* packet = new YourServiceMessage("YourService Packet");
                packet->setMessageType(message_type.c_str());
                packet->setVehicleId(vehicle_id.c_str());
                packet->setSensorId(sensor_id.c_str());
                packet->setInitialTimestamp(initial_timestamp);
                packet->setDistance(distance);
                packet->setX(coord.y);
                packet->setY(coord.x);
                packet->setByteLength(42);  // optional, for simulation traffic size
            }
        }
    }

Explanation of the Last Block
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* The inner ``do { ... } while (true);`` loop:
    * Uses ``socket.recv(..., zmq::recv_flags::dontwait)`` to read all available ZMQ messages without blocking.
    * For each message, it parses the JSON payload and extracts:
        * ``message_type`` - type of message (e.g., obstacle data),
        * ``vehicle_id`` - identifier of the vehicle in SUMO/CARLA,
        * ``sensor_id`` - identifier of the sensor on that vehicle,
        * ``initial_timestamp`` - timestamp from CARLA,
        * ``distance`` - obstacle distance reported by the sensor.
    * It prints a debug line with the obstacle ID, vehicle, timestamp, and distance.

* After consuming all pending messages, the code retrieves the vehicle geometry and pose:
    * ``vehicle_length`` and ``vehicle_width`` from the Artery vehicle model,
    * ``vehicle_angle`` from ``getHeading().radian()``,
    * ``cur_pos_center`` as the world coordinate of the vehicle center.

* The variable ``depth`` is set to the reported ``distance`` and is interpreted as the distance of the obstacle in front of the vehicle along its longitudinal axis.

* The detection is first expressed in **vehicle-local coordinates**:
    * ``det_x_v`` is half the vehicle length plus the obstacle depth (distance ahead of the center),
    * ``det_y_v`` is zero, meaning the detection lies on the vehicle's longitudinal axis.

* The pair ``(det_x_v, det_y_v)`` is then rotated into **world coordinates** using the vehicle heading:
    * ``det_dx`` and ``det_dy`` give the obstacle's offset relative to the vehicle center in the global coordinate system.

* Two equivalent methods are shown:
    * ``detection_coord`` adds the rotated offset directly to ``cur_pos_center``.
    * ``coord`` first defines a sensor located at the front of the vehicle, then adds a straight-ahead offset (``depth``) to locate the detection point.

* Finally, a :cpp:type:`YourServiceMessage` is created and filled:
    * The message fields are populated with the parsed JSON values.
    * ``distance`` is stored as received.
    * The world coordinates of the detection are stored in ``x`` and ``y`` using ``coord``.
    * The optional byte length field is set to 42 to simulate network payload size.

This packet can then be forwarded by the service to other vehicles, which can process the obstacle information using Artery's normal message-handling mechanisms.

Scenario Development
--------------------

After compiling the custom service successfully, the next step is to create and configure the simulation scenario in which it will run.  
Follow the detailed setup guide at http://artery.v2x-research.eu/custom-simulation/ as a reference for the standard Artery workflow.

For this framework, the scenario directory will be located at:

``Path/To/Artery/scenarios/framework``

Obtaining SUMO Files
~~~~~~~~~~~~~~~~~~~~

To ensure compatibility between CARLA and Artery, you need the corresponding SUMO configuration files.  
CARLA already provides ready-to-use SUMO projects for several towns in:

``Path/To/CARLA/Co-Simulation/Sumo/examples``

The default examples include **Town01**, **Town05**, and **Town06**.  
If you wish to work with another CARLA map, CARLA provides a utility to automatically generate the SUMO network files.  
Refer to: https://carla.readthedocs.io/en/latest/adv_sumo/#create-the-sumo-net for the required steps.

Furthermore, SUMO needs to know which vehicles it is importing from CARLA.  
Follow the steps in https://carla.readthedocs.io/en/latest/adv_sumo/#create-carla-vtypes to define CARLA vtypes for SUMO.

For demonstration purposes, the provided examples will suffice.  
Copy the folder ``examples`` from ``Path/To/CARLA/Co-Simulation/Sumo/`` and paste it into your new directory:

``Path/To/Artery/scenarios/framework``

Scenario Configuration
~~~~~~~~~~~~~~~~~~~~~~

Next, create an ``omnetpp.ini`` file inside the scenario directory.  
This configuration defines how Artery and SUMO will launch and synchronize during the simulation:

.. code-block:: cfg
    :linenos:
    :caption: omnetpp.ini

    [General]
    network = artery.inet.World # Avoid Veins
    *.traci.launcher.typename = "PosixLauncher" # Launch SUMO
    *.traci.launcher.port = 9998 # SUMO port 
    *.traci.launcher.extraOptions = "--num-clients 2" # 2 clients: run_synchronization.py and Artery
    *.traci.launcher.sumocfg = "examples/Town05.sumocfg" # SUMO files
    *.node[*].middleware.updateInterval = 0.05s # Match CARLA tick rate 
    *.node[*].middleware.datetime = "2025-11-06 13:30:00" # Start date and time
    *.node[*].middleware.services = xmldoc("services.xml") # XML file listing YourService

Defining Services
~~~~~~~~~~~~~~~~~

The XML file listed above defines which services will be active in this scenario.  
Place it in the same directory as ``omnetpp.ini`` and name it ``services.xml``:

.. code-block:: xml
    :linenos:
    :caption: services.xml

    <?xml version="1.0" encoding="UTF-8"?>
    <services>
        <service type="artery.application.YourService" name="YS">
            <listener port="2024"/>
        </service>
    </services>

Vehicle Population
~~~~~~~~~~~~~~~~~~

CARLA also provides route definition files that populate each town with traffic participants.  
For Town05, the file is located at:

``examples/rou/Town05.rou.xml``

By default, this file defines around 200 vehicles.  
To simplify testing and ensure correct behavior during initial validation, reduce the number of vehicles to two.  
Your modified ``Town05.rou.xml`` should look like this:

.. code-block:: xml
    :linenos:
    :caption: rou.xml

    <?xml version='1.0' encoding='UTF-8'?>
    <routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
        <vehicle id="0" type="vehicle.bh.crossbike" depart="0.00">
            <route edges="43.0.00 23.0.00 -52.0.00 49.0.00 7.0.00 6.0.00"/>
        </vehicle>
        <vehicle id="1" type="vehicle.micro.microlino" depart="1.00">
            <route edges="33.0.00 -39.0.00 25.0.00 -6.0.00"/>
        </vehicle>
    </routes>

For more details on SUMO route configuration, visit:  
https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html

Summary
-------

In this section, you:

* Enabled ZeroMQ support in the Artery build system.
* Created a custom Artery service (YourService) to subscribe to CARLA sensor data.
* Defined the :file:`YourServiceMessage.msg` message format and integrated it into the build.
* Implemented ZMQ initialization, subscription, cleanup, and a destructor for safe shutdown.
* Parsed JSON messages from CARLA and computed obstacle positions in world coordinates.
* Created a scenario directory, INI configuration, services XML, and a simplified SUMO route file.

With these steps, Artery is now ready to receive and process obstacle data from CARLA, completing the network-simulation side of the framework.

Continue
~~~~~~~~

You can now proceed to running the full co-simulation:

* :doc:`run_simulation` - how to launch CARLA, SUMO, and Artery together.
