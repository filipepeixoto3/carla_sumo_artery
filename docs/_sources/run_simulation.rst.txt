Run Simulation
==============

Once all configurations are complete, the final step is to launch the full co-simulation.  
This stage starts CARLA, loads the appropriate map, initializes SUMO, and runs the Artery scenario that communicates through ZeroMQ.

Follow the commands below in **separate terminals**, in the order presented.

Launching CARLA
----------------

.. code-block:: bash
    :linenos:
    :caption: Start the CARLA simulator

    cd ~/Path/to/CARLA
    ./CarlaUE4.sh

Switching CARLA to Town05
-------------------------

.. code-block:: bash
    :linenos:
    :caption: Load Town05 and initialize SUMO settings

    cd ~/Path/to/CARLA/PythonAPI/util
    python3 config.py --map Town05

Running Artery
--------------

.. code-block:: bash
    :linenos:
    :caption: Start Artery with the configured framework scenario

    cd ~/Path/to/Artery/
    cmake --build build --target run_framework

Explanation
-----------

1. The first command launches the **CARLA simulator**.  
2. The second loads **Town05**, applying the SUMO configuration required for synchronized co-simulation.  
3. The final command runs **Artery** with the newly implemented service and scenario.

When executed correctly, **CARLA, SUMO, and Artery will run in synchronization**, exchanging obstacle sensor data through the ZMQ bridge.

Video Demonstration
-------------------

Below is an example video showing the full co-simulation running.  
By executing:

``Path/To/CARLA/PythonAPI/examples/manual_control.py``

A small Pygame window appears (bottom corner) that allows manual driving of a CARLA vehicle.  
The forward-facing ray in the video is the obstacle sensor added for debugging.  
The top-right PowerShell window shows the output of ``run_synchronization.py`` running in parallel.

.. raw:: html

   <video width="700" controls>
       <source src="_static/framework_vid.mp4" type="video/mp4">
       Your browser does not support the video tag.
   </video>

Summary
~~~~~~~

In this section, you:

* Started **CARLA**, selected the correct town, and prepared SUMO integration  
* Launched **Artery** with the custom ZeroMQ-enabled service  
* Observed all three simulators running in synchronized co-simulation  
* Validated that sensor data travels from CARLA → SUMO → Artery in real time  

This completes the execution stage of the **CARLA–SUMO–Artery** framework.  
With the system running, you can now experiment with:

* different vehicle densities  
* alternative SUMO route configurations  
* modified sensor setups  
* new Cooperative Perception message types  
* or extended Artery services  

You now have a full, functioning pipeline capable of evaluating cooperative ITS and perception-sharing applications in a realistic 3D environment.
