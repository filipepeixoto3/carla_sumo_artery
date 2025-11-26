.. Framework Manual documentation master file, created by
   sphinx-quickstart on Thu Sep 18 09:14:01 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.
   

CARLA + SUMO + OMNeT++ Framework Documentation
===================================================

Welcome to the CAr Learning To Act (CARLA) + Simulation of Urban MObility (SUMO) + (Objective Modular Network Testbed in C++) OMNeT++ co-simulation strategy.

This work combines CARLA's sensing capabilities with OMNeT++'s network communications, more specifically, Vehicular ad-Hoc Networks (VANETs), either though Vehicles In Network Simulation (Veins) or Artery.

The goal is to be able to leverage CARLA's 3D world and migrate sensor's outputs to the network simulator in order to implement VANET applications and services based on Collective Perception.

Since SUMO has a bridge between both CARLA and Veins/Artery, this entity will be responsible for the traffic generation of the scenarios.
For the bridge between CARLA and Artery a ZeroMQ bridge is implemented between the two.

.. toctree::
   :maxdepth: 1
   :caption: Contents:
   
   prerequisites
   carla_sumo_cosim
   artery_setup
   run_simulation
   



.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`
