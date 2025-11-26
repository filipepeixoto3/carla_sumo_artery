Prerequisites
=============

This section covers all installations required to prepare the environment for the **CARLA–SUMO–Artery** co-simulation framework.  
All instructions assume an Ubuntu Linux environment, since Artery does not support native Windows builds.

CARLA Installation
------------------

For complete documentation, see: https://carla.readthedocs.io/en/latest/

Installation Steps
~~~~~~~~~~~~~~~~~~
1. Download CARLA (version 0.9.15 recommended):  
   https://github.com/carla-simulator/carla/releases/tag/0.9.15

2. Extract the package to a preferred directory.

3. Ensure Python is between versions **3.7** and **3.12**.

4. Ensure your Python package manager (`pip`) is version **20.3** or higher.

5. Install additional dependencies:

   .. code-block:: bash

       apt-get install libomp5
       pip install pygame numpy

6. Test the installation:

   .. code-block:: bash

       ./CarlaUE4.sh


SUMO Installation
-----------------

Full documentation: https://eclipse.dev/sumo/

Installation Steps
~~~~~~~~~~~~~~~~~~
1. Install SUMO:

   .. code-block:: bash

       apt-get install sumo sumo-tools sumo-doc

   Check that the environment variable was created:

   .. code-block:: bash

       echo $SUMO_HOME
       # Expected output: /usr/share/sumo

2. Install Python tools for SUMO:

   .. code-block:: bash

       pip install -r $SUMO_HOME/tools/requirements.txt

3. Verify the installation:

   .. code-block:: bash

       sumo --version


OMNeT++ Installation
--------------------

OMNeT++ is used to run Artery. Recommended version: **OMNeT++ 5.7.1**, as it is the latest compatible with Artery's stable builds.

Official download page: https://omnetpp.org/download/

Installation Steps
~~~~~~~~~~~~~~~~~~
1. Download OMNeT++ **5.7.1**:

   https://github.com/omnetpp/omnetpp/releases/tag/omnetpp-5.7.1

2. Extract the package:

   .. code-block:: bash

       tar xf omnetpp-5.7.1-linux-x86_64.tgz
       cd omnetpp-5.7.1

3. Install dependencies:

   .. code-block:: bash

       sudo apt-get update
       sudo apt-get install build-essential gcc g++ bison flex \
            perl python3 qtbase5-dev libqt5opengl5-dev tcl-dev tk-dev \
            libxml2-dev zlib1g-dev default-jre doxygen graphviz

4. Configure and compile OMNeT++:

   .. code-block:: bash

       . setenv
       ./configure
       make -j$(nproc)

5. Launch OMNeT++:

   .. code-block:: bash

       ./omnetpp


ZeroMQ Installation
-------------------

ZeroMQ is used as the communication layer between CARLA and Artery.

Full reference: https://zeromq.org/download/

Installation Steps
~~~~~~~~~~~~~~~~~~
1. Install the ZeroMQ development package:

   .. code-block:: bash

       apt-get install libzmq3-dev

2. Verify the installation:

   .. code-block:: python

       import zmq
       print(zmq.zmq_version())


Summary
-------

In this section you:

* Installed **CARLA**, **SUMO**, **OMNeT++**, and **ZeroMQ**  
* Prepared a Linux-based environment compatible with **Artery**  
* Verified that all required tools run correctly  
* Completed all foundational steps needed to proceed with the framework

Continue
~~~~~~~~

You can now continue to the next part of the manual:

* :doc:`carla_sumo_cosim` — to understand how CARLA and SUMO communicate  
* :doc:`artery_setup` — to configure Artery and integrate the ZMQ service
