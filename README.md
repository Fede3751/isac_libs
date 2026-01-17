# Isac Libs - ROS & Gazebo Libraries for Integrated Sensing and Communication

A set of open-source Integrated Sensing and Communication (ISAC) libraries for ROS and Gazebo.

## Note:
This library is currently undergoing a submission to **INFOCOM 2026**.
We plan to update the library in these coming months, but it would be unfair to have reviews based on content added after the submission.\
\
For full transparency, we are keeping track of the latest commits before the submissions here:
- The latest version commit before the **Demo** submission can be found here: <a href="https://github.com/Fede3751/isac_libs/tree/9127274f0eadde6176ad415e619054e3bf676367"> Commit 9127274</a>
- The latest version commit before the **Workshop** submission can be found here: <a href="https://github.com/Fede3751/isac_libs/tree/f0b8a838ad6423c4cc859607ad83646832dac4de"> Commit f0b8a83</a>


## Overview & Setup

This repository hosts all of the source code of the framework. The framework is built on **ROS 2 Humble** and **Gazebo (Ignition)**; and is designed to support research and prototyping of **ISAC** applications.

The toolkit provides:

- A **custom Gazebo Sensor Plugin** that emulates ISAC wireless signal propagation and sensing effects.
- A **ROS 2 control layer** for implementing application logic, mobility, and network behavior.


To use the libraries, just compile and source like a normal ROS workspace

```bash
mkdir src
git clone https://github.com/Fede3751/isac_libs.git src
colcon build
source ./install/local_setup.bash
```

This set up env variables for both ROS packages and Gazebo models and plugins.

---

## ROS 2 Workspace & Packages Organization

The framework is organized into multiple ROS 2 packages, which are listed and described here:

### isac_libs_main

Main package. Contains the **ISACDeviceController** ROS2 Node Class, which can be extended to interface with an ISAC Device in a Gazebo simulation.
Additionally, some launch utilities are given to set up easily a simulation:

##### launch_utils.py
Contains some function to support a ROS 2 Python Launch File.

```
create_gazebo_launch_description(world_name)
```
Used to return a list of ROS 2 Launch Nodes which start Gazebo along with important bridges for world clock and world control.
The world_name launched should be found inside ```src/isac_libs_resources/worlds``` before compilation. Due to some limitation with ROS launch substitutions, world files are expected to be without the .sdf extension.

```
create_isac_device_launch_description(...)
```
Used to launch a ROS 2 node which extends an ISACDeviceController class, it automatically set ups all namespaces correctly, and starts bridges for the topics used by the device (tx_data, rx_data, cmd_vel, odometry)

```
spawn_sdf(...)
```
Used to dinamically spawn a Gazebo sdf model at startup. The function loads an sdf from a file and changes every occurency of the model name with name_id.

  The function looks for sdf files inside ```src/isac_libs_resources/models```. All the models share a common structure, with a folder named after the model, and a ```model.sdf``` file inside. Some example pre-defined models are available.
Note that this function is currently not time safe and may fail to spawn some devices if overloaded. Corrections will be coming in next iterations of the library. For the time being, please increase the timeout timer used in the function if you notice that some models fail to appear.

##### event_scheduler.py
Contains the **EventScheduler** class. This class serves as a Gazebo-synced clock utility, which is to be used for events that need to happen in actual simulation time; as Gazebo may run slower or faster than real time depending on simulation complexity and world parameters.
Upon creation, its method _.routine_ should be linked to the subscriber callback of Gazebo Clock topic, like this:

```python
from isac_libs_main.utils.event_scheduler include EventScheduler

# inside the __init__ of your class:

self.event_scheduler = EventScheduler()
self.create_subscription(
  Clock,
  f"/world/{WORLD_NAME}/clock",
  self.event_scheduler.routine,
  10
)
```
Events can then be scheduled like this:
```python
self.event_scheduler.schedule_event(timeout, callback_function, repeat=True|False, args=func_args)
```

##### math_utils.py
Just a collection of math function/utilities used sometimes across the library.

### isac_libs resources
Contains all the resources used by Gazebo. **ISACAntenna** and **ISACAntennaSystem** plugins are stored here. Please refer to their documentation in the next section.

### isac_libs_interfaces
Contains ROS interfaces used by the MEC example.

### isac_libs_mec_example
Contains the MEC Example demo. 
The demo can be started directly with the following launch file:

```
ros2 launch isac_libs_mec_example main.launch.py
```

The default simulation spawns 1 edge device, 3 relays, and 1 agent user. Parameters can be given to change multiple simulation settings.

### isac_libs_sensing_example
Contains the Passive Sensing Example demo. 
The demo can be started directly with the following launch file:

```
ros2 launch isac_libs_sensing_example main.launch.py
```

---

## Using the ISACAntenna Plugin


In order for the plugin to to work, it is necessary to add the **ISACAntennaSystem** system plugin to the Gazebo sdf world.

```xml
<plugin
  filename="ISACAntennaSystem"
  name="custom::ISACAntennaSystem">
</plugin>
```

After that, any model inside the simulation can use its own **ISACAntenna** Sensor, which can be attached to a link of the model and configured with multiple parameteres as follows:


```xml
<sensor name="ISACAntennaSensor" type="custom" ignition:type="ISACAntenna">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <ignition:ISACAntenna>
    <tx_power_dBm>15</tx_power_dBm>
    <path_loss_exponent>3.2</path_loss_exponent>
    <fading_std>4.0</fading_std>
    <bandwidth_Hz>1000000</bandwidth_Hz>
    <noise_figure_dB>5.0</noise_figure_dB>
  </ignition:ISACAntenna>
</sensor>
```

Models inside ```isac_libs_resources``` already come with their ISACAntenna sensor configured, with different parameters set depending on the device.


## Future Work & Directions

The work of ISAC Libs is still ongoing, with new features planned mainly for the Gazebo plugin.

- Update of the 2D interface package. Currently left behind for a focus on offline evaluation. We feel that Gazebo needs an online evaluation tool.
- Support for directional antennas, with ad-hoc models advertising ```cmd_vel``` to move their orientations
- Support for Reconfigurable Intelliget Surfaces (RIS) interfaces
