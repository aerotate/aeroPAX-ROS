# Dronecan - ROS2 integration for aeroPAX battery

## Seting up the USB2CAN adapter

This setup is done with USB2CAN adapter from Innomaker. For setting up the driver for can adapter, can-utils library is needed:

```
sudo apt-get install can-utils
```

To inititate the can adapter the following command in terminal has to be executed:

```
sudo ip link set can0 up type can bitrate 1000000
```
To verify that the connection was successful, the command `ifconfig -a` should display the can0 interface.

To see more information about USB2CAN module :

```
sudo demsg
```

## ROS2 package

Clone the repository and build the aeroPAX-ROS package :

```
git clone https://github.com/aerotate/aeroPAX-ROS.git
cd aeroPAX-ROS
colcon build
```

For running the node :

```
source /opt/ros/version_name/setup.bash
source /install/setup.bash
ros2 run drone_can dronecan_listener.py
```

Then open another terminal and after sourcing ros2 again: 

```
ros2 topic echo /BatteryState
```


## Parameters

Without an assigned Node_ID can nodes do not transmit any data. Therefore, `Dronecan_listener node` first inititates a can node and assigns a dynamical `node_id` to the battery Dronecan node. It then publishes BatteryState message on `/BatteryState` topic. This topic provides real-time information about the battery, including various parameters such as voltage, current, state of charge, and more. The transfarred values are shown below : 
 
- **frame_id**: The reference frame identifier for the data.
- **voltage**: The battery voltage in volts (V).
- **current**: The current drawn by the system in amperes (A).
- **state_of_charge**: The battery's state of charge in percentage (%).
- **temperature**: The battery temperature in degrees Celsius (°C).
- **can_node_id**: The CAN bus node identifier assigned to the battery.
- **battery_id**: The unique identifier for the battery.
- **model_instance_id**: Battery model ID. 

In the context of ROS messages, the values for `battery_id` and `model_instance_id` are not defined. For this reason, `location` and `serial_number` parameters are used to define those values. `Location = Battery_id, Serial_number = model_instance_id` .








