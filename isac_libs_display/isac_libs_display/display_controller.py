import sys
import time
import json
from threading import Thread
import dearpygui.dearpygui as dpg

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from isac_libs_mec_example.structs.cache_strategy import CacheType


from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String

from isac_libs_interfaces.msg import SensorPositions, BSQuery


WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
COLUMN_INFO_WIDTH = 500
SIMPLIFIED_SENSOR_SIM = False

CACHES_NO = len(CacheType)

class Display(Node):
    def __init__(self):
        super().__init__('display')

        self.last_queried_position = Point()
        self.sensors = {}
        self.edge_devices = {}
        self.agents = {}
        self.caches = {}


        self.declare_parameters(
            namespace="",
            parameters=[
                ("edge_devices", 3),
                ("sensors", 3),
                ("agents", 1)
            ]
        )

        self.no_sensors = (
            self.get_parameter("sensors").get_parameter_value().integer_value
        )

        self.no_edge_devices = (
            self.get_parameter("edge_devices").get_parameter_value().integer_value
        )

        self.no_agents = (
            self.get_parameter("agents").get_parameter_value().integer_value
        )



        self.timer_period = 0.1  # 10 Hz
        #self.timer = self.create_timer(self.timer_period, self.update_position)

        if SIMPLIFIED_SENSOR_SIM:
            self.create_subscription(
                    SensorPositions,
                    "/NetworkManager/SensorPositions",
                    self.store_simple_sensor_position,
                    10,
                )
        
        else:
            for i in range(self.no_sensors):
                self.create_subscription(
                    Odometry,
                    f"/ActiveSensor_{i}/odometry",
                    lambda odometry_msg, sensor_id=i: (
                        self.store_sensor_position(sensor_id, odometry_msg)
                    ),
                    10,
                )

        for i in range(self.no_edge_devices):
            self.create_subscription(
                Odometry,
                f"/EdgeDevice_{i}/odometry",
                lambda odometry_msg, sensor_id=i: (
                    self.store_edge_device_position(sensor_id, odometry_msg)
                ),
                10,
            )

            self.create_subscription(
                String,
                f"/EdgeDevice_{i}/cache_content",
                lambda cache_msg, edge_device_id=i: (
                    self.update_cache_content(edge_device_id, cache_msg)
                ),
                10
            )

        for i in range(self.no_agents):
            self.create_subscription(
                Odometry,
                f"/Agent_{i}/odometry",
                lambda odometry_msg, agent_id=i: (
                    self.store_target_position(agent_id, odometry_msg)
                ),
                10,
            )

        self.create_subscription(
            BSQuery,
            "/BaseStation/LastQueriedPosition",
            self.store_last_queried_position,
            100
        )


        self.start_gui()
        self.get_logger().info("Gui started")


    def store_simple_sensor_position(self, sensor_positions: SensorPositions):
        
        for sensor_id, sensor_pos in enumerate(sensor_positions.positions):
            if not sensor_id in self.sensors:
                self.sensors[sensor_id] = {"position": Point()}

            self.sensors[sensor_id]["position"] = sensor_pos
        

    def store_sensor_position(self, sensor_id, odometry: Odometry):

        if not sensor_id in self.sensors:
            self.sensors[sensor_id] = {"position": Point()}

        self.sensors[sensor_id]["position"] = odometry.pose.pose.position


    def store_target_position(self, agent_id, odometry: Odometry):

        if not agent_id in self.agents:
            self.agents[agent_id] = {"position": Point()}

        self.agents[agent_id]["position"] = odometry.pose.pose.position


    def store_edge_device_position(self, edge_device_id, odometry: Odometry):

        if not edge_device_id in self.edge_devices:
            self.edge_devices[edge_device_id] = {"position": Point()}

        self.edge_devices[edge_device_id]["position"] = odometry.pose.pose.position

    def update_cache_content(self, edge_device_id, cache_msg: String):

        self.caches[edge_device_id] = json.loads(cache_msg.data)


    def store_last_queried_position(self, query : BSQuery):

        self.last_queried_position = query.pos
        
    def start_gui_function(self):


        dpg.create_context()
        dpg.create_viewport(title='ISAC Libs 2D View', width=WINDOW_WIDTH+COLUMN_INFO_WIDTH+25, height=WINDOW_HEIGHT+35)

        while len(self.sensors) < self.no_sensors or len(self.edge_devices) < self.no_edge_devices or len(self.agents) < self.no_agents:
            self.get_logger().info(f"Waiting for edge devices and sensor to spawn")
            time.sleep(1)
        
        
        with dpg.window(label="", width=WINDOW_WIDTH+COLUMN_INFO_WIDTH+25, height=WINDOW_HEIGHT+35):
                
            with dpg.group(horizontal=True):

                with dpg.plot(label="Field View", width=WINDOW_WIDTH, height=WINDOW_HEIGHT, tag="plot"):
                    
                    dpg.add_plot_axis(dpg.mvXAxis, label="X-Axis", tag="X-Axis")
                    
                    with dpg.plot_axis(dpg.mvYAxis, label="Y-Axis", tag="Y-Axis"):
                        
                        dpg.add_scatter_series(
                            [sensor["position"].x for sensor in self.sensors.values()],
                            [sensor["position"].y for sensor in self.sensors.values()],
                            label="Moving Points",
                            id="sensor_positions",
                            tag="sensor_positions",
                        )

                        dpg.add_scatter_series(
                            [edge_device["position"].x for edge_device in self.edge_devices.values()],
                            [edge_device["position"].y for edge_device in self.edge_devices.values()],
                            label="EdgeDevices",
                            id="edge_device_positions",
                            tag="edge_device_positions"
                        )

                        dpg.add_scatter_series(
                            [target["position"].x for target in self.agents.values()],
                            [target["position"].y for target in self.agents.values()],
                            label="Targets",
                            id="target_positions",
                            tag="target_positions"
                        )

                        dpg.add_scatter_series(
                            [self.last_queried_position.x],
                            [self.last_queried_position.y],
                            label="Last Queried Position",
                            id="last_queried_pos",
                            tag="last_queried_pos"
                        )


                        # Add Labels for each point
                        for label, sensor in self.sensors.items():
                            x, y = sensor["position"].x, sensor["position"].y
                            dpg.draw_text((x-0.5, y+4.5), label, color=(76, 114, 176, 255), size=2, parent="plot")
                        for label, edge_device in self.edge_devices.items():
                            x, y = edge_device["position"].x, edge_device["position"].y
                            dpg.draw_text((x-0.5, y+4.5), label, color=(221, 132, 82, 255), size=2, parent="plot")


                with dpg.child_window(width=COLUMN_INFO_WIDTH, height=WINDOW_HEIGHT, autosize_y=False, border=True):

                    cols = CACHES_NO

                    for row in range(4):

                        with dpg.group():
                            dpg.add_text(f"Edge Device {row}:")    
                            with dpg.child_window(width=-1, height=160, border=True, tag=f"info_row_{row}"):
                                with dpg.group(horizontal=True):
                                    for cache in CacheType:
                                        with dpg.child_window(width=(COLUMN_INFO_WIDTH-((cols+3)*8 + 1))/cols, height=-1, border=True, tag=f"cache_{row}_{cache.name}_container"):
                                            block_tag = f"cache_{row}_{cache.name}"
                                            dpg.add_text(f"{cache.name}", tag=block_tag)
                                        
                            dpg.add_spacer(height=5)
 

        dpg.setup_dearpygui()
        dpg.show_viewport()

        while dpg.is_dearpygui_running():

            dpg.set_value("sensor_positions", [[sensor["position"].x for sensor in self.sensors.values()], [sensor["position"].y for sensor in self.sensors.values()]])
            dpg.set_value("target_positions", [[target["position"].x for target in self.agents.values()], [target["position"].y for target in self.agents.values()]])
            dpg.set_value("last_queried_pos", [[self.last_queried_position.x], [self.last_queried_position.y]])
            for row in self.edge_devices:
                for cache in CacheType:
                    if row in self.caches:
                        formatted = "\n".join(str(v) for v in self.caches[row][cache.name])
                        dpg.set_value(f"cache_{row}_{cache.name}", f"{cache.name}\n\n{formatted}")
            dpg.fit_axis_data("X-Axis")
            dpg.fit_axis_data("Y-Axis")
            dpg.render_dearpygui_frame()
            time.sleep(0.1)


        dpg.destroy_context()
        dpg.stop_dearpygui()


    def start_gui(self):
        self.display_thread = Thread(target=self.start_gui_function)
        self.display_thread.start()
        

    def update_position(self):
        """Update the robot's position based on the velocities."""

    def destroy_node(self):
        dpg.stop_dearpygui()  # Tells the event loop to stop
        dpg.destroy_context()  # Closes the window
        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    display = Display()

    executor.add_node(display)
    executor.spin()

    # Destroy the node explicitly
    executor.shutdown()
    display.destroy_node()
    rclpy.shutdown()



def shutdown_handler(sig, frame):
    print("Shutting down DearPyGui...")
    dpg.stop_dearpygui()  # Tells the event loop to stop
    dpg.destroy_context()  # Closes the window
    sys.exit(0)


if __name__ == '__main__':
    # signal.signal(signal.SIGTERM, shutdown_handler)
    # signal.signal(signal.SIGKILL, shutdown_handler)
    # signal.signal(signal.SIGINT, shutdown_handler)
    main()
