import rclpy
import dronecan
import dronecan.app.dynamic_node_id
import dronecan.app.node_monitor

from rclpy.node import Node
from sensor_msgs.msg import BatteryState
# Dronecan message definitions are take from this website
# https://dronecan.github.io/Specification/7._List_of_standard_data_types/

class DronecanListenerNode(Node):
    def __init__(self):
        super().__init__('dronecan_listener')

        # Creating a publisher to publish BatteryState message on the '/BatteryState' topic
        self.battery_publisher = self.create_publisher(BatteryState, '/BatteryState', 10)

        # Initiating a dronecan node 
        self.cannode = dronecan.make_node('can0', node_id=65, bitrate=1000000)

        self.cannode_monitor = dronecan.app.node_monitor.NodeMonitor(self.cannode)

        # dynamic node id allocator 
        self.dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.cannode, self.cannode_monitor)

        # Add a handler to process incoming BatteryInfo messages
        self.cannode.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.power_callback)
        self.timer = self.create_timer(0.02, self.timer_callback)

    # callback function for processing Battery info 
    def timer_callback(self):
        self.cannode.spin(0)

    def power_callback(self, event):
        
        battery_state = BatteryState()

        battery_state.header.stamp = self.get_clock().now().to_msg()
        battery_state.header.frame_id = 'aeroPAX6'
    
        battery_state.voltage = event.message.voltage
        battery_state.current = event.message.current
        battery_state.temperature = round(event.message.temperature - 273,2)
        battery_state.percentage = float(event.message.state_of_charge_pct)
        battery_state.serial_number = str(event.message.model_instance_id)
        battery_state.location = str(event.message.battery_id)
        # remove the comment if thosee values are available

        # battery_state.charge = float(event.message.hours_to_full_charge)
        # battery_state.capacity = float(event.message.remaining_capacity_wh)

        battery_state.charge = float('nan')
        battery_state.capacity = float('nan')
        battery_state.design_capacity = float('nan')
        battery_state.present = bool(True) 


        self.battery_publisher.publish(battery_state)


    def destroy_node(self):
        self.cannode.close() 
        self.allocator.close()
        self.node_monitor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DronecanListenerNode()
    try:
        # Spin the node to keep it running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and destroy the node on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()