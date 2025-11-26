# Project Overview

This repository contains the components needed to integrate and document the
CARLA–SUMO–OMNeT++ Cooperative Simulation Framework.

## Repository Structure

### `artery/`
Contains the files required to be added to the **Artery** project.  
These files extend Artery with the modules, services and configurations used in this framework.

### `carla/`
Contains the files required to be added to the **CARLA** project.  
These include modifications and integration points used to enable synchronized co-simulation.

### `docs/`
Contains the **Sphinx-generated manual**, ready to be deployed (e.g., GitHub Pages / Google Pages).  
This is the compiled output of the documentation. Access here: https://filipepeixoto3.github.io/carla_sumo_artery/

### `frame_manual.zip`
Contains the **source Sphinx folders** used to generate the documentation present in `docs/`.  
Unzip and build to regenerate the full manual.
