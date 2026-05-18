#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import dearpygui.dearpygui as dpg
import threading
import time

from rclpy.executors import MultiThreadedExecutor

class ISACSignalDisplay(Node):
    def __init__(self):
        super().__init__("isac_signal_display")

        # Subscribe to the telemetry topic
        self.subscription = self.create_subscription(
            String,
            "/Agent_1/rx_data",
            self.rx_callback,
            10
        )

        # Buffer to store latest telemetry
        self.telemetry_data = {
            "snr_dB": [],
        }


        self.start_time = time.time()


        self.gui_thread = threading.Thread(target=self.start_gui, daemon=True)
        self.gui_thread.start()

    def rx_callback(self, msg: String):
        try:
            
            data_json = json.loads(msg.data)
            telemetry = data_json["telemetry"]
            t = time.time() - self.start_time

            # Store latest values
            self.telemetry_data["snr_dB"].append((t, telemetry["snr_dB"]))

            # Keep only last N points
            max_points = 100
            for key in self.telemetry_data:
                self.telemetry_data[key] = self.telemetry_data[key][-max_points:]

        except Exception as e:
            self.get_logger().warn(f"Failed to parse telemetry: {e}")

    def start_gui(self):

        dpg.create_context()
        dpg.create_viewport(title="ISAC Signal Display", width=800, height=250)
        

        with dpg.window(label="ISAC Signal Telemetry", width=800, height=250):
            dpg.add_text("Live Signal Data for Agent_1")

            with dpg.plot(label="", height=150, width=-1) as rx_plot:

                x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", parent=rx_plot, tag="snr_dB_x")
                y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="snr_dB", parent=rx_plot, tag="snr_dB_y")
                self.snr_series = dpg.add_line_series([], [], label="snr_dB", parent=y_axis)



        dpg.setup_dearpygui()
        dpg.show_viewport()

        while dpg.is_dearpygui_running():

            for t, y in self.telemetry_data["snr_dB"]:
                dpg.set_value(self.snr_series, ([x[0] for x in self.telemetry_data["snr_dB"]],
                                                        [x[1] for x in self.telemetry_data["snr_dB"]]))

            dpg.fit_axis_data("snr_dB_x")
            dpg.fit_axis_data("snr_dB_y")


            dpg.render_dearpygui_frame()
            time.sleep(0.1)
        dpg.destroy_context()




def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    display = ISACSignalDisplay()
    executor.add_node(display)

    executor.spin()

    executor.shutdown()
    display.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
