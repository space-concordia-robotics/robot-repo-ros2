# ruff: noqa: D100, D101, D102, D103, D107
import array
import textwrap
import time
from time import sleep

import numpy
import rclpy
import seabreeze
from numpy.typing import NDArray
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rover_msgs.action import ScienceSpectrometerScan
from seabreeze.spectrometers import Spectrometer
from seabreeze.spectrometers import list_devices as list_oceanoptics_devices
from seabreeze.types import SeaBreezeDevice


class SpectrometerService(Node):
    __scan_action_server: ActionServer
    __spectrometer: Spectrometer

    def __init__(self):
        super().__init__("science_payload")

        devices: list[SeaBreezeDevice]

        # TODO 2026-05-28 (Will Free): add config parameter for this
        seabreeze.use("pyseabreeze")

        while True:
            devices = list_oceanoptics_devices()

            if len(devices) > 0:
                break
            self.get_logger().info("Waiting for Ocean Optics spectrometer device...")

            sleep(5)

        self.get_logger().info(f"Found {len(devices)} Ocean Optics spectrometer devices")
        if len(devices) != 1:
            self.get_logger().error("More than 1 spectrometer found, unsure which devices to use. Exiting.")
            rclpy.shutdown()

        device = devices[0]

        self.__spectrometer = Spectrometer(device)

        features: dict[str, list[str]] = {
            feature: [item.identifier for item in feature_items]  # ty:ignore[not-iterable]
            for feature, feature_items in self.spectrometer.features.items()
        }

        features_string = {feature: "\n".join(f"* {item}" for item in items) for feature, items in features.items()}
        features_string = {feature: textwrap.indent(items, "  ") for feature, items in features_string.items()}
        features_string = "\n".join([f"* {feature}" + "" if items == "" else f"\n{items}" for feature, items in features_string.items()])

        # TODO 2026-05-10 (Will Free): print hardware revision & firmware revision here (in revision feature)

        # TODO 2026-05-10 (Will Free): sigh, multiline strings aren't un-indented automatically so I need to do this garbage.
        #  and it's annoying to have to wrap the entire thing in textwrap.dedent.
        integration_micros_limits = self.spectrometer.integration_time_micros_limits
        self.get_logger().info(f"""Found spectrometer:
* model: {self.spectrometer.model}
* serial number: {self.spectrometer.serial_number}
* wavelengths: {min(self.spectrometer.wavelengths())} nm - {max(self.spectrometer.wavelengths())} nm
* pixels: {self.spectrometer.pixels}
* integration time hard limits: {integration_micros_limits[0]} µs - {integration_micros_limits[1]} µs
* features:
{textwrap.indent(features_string, "  ")}
""")
        self.__scan_action_server = ActionServer(
            self,
            ScienceSpectrometerScan,
            "/rover/science/scan",
            self.execute_scan,
        )

    def execute_scan(self, goal_handle: ServerGoalHandle) -> ScienceSpectrometerScan.Result:
        self.get_logger().info("Triggered spectrometer scan")

        # TODO 2026-05-10 (Will Free): add field to request for light sources
        request: ScienceSpectrometerScan.Goal = goal_handle.request

        # TODO 2026-05-12 (Will Free): add code to turn light on before measurement & off afterwards?

        if request.integration_time_ms != 0:
            self.spectrometer.f.spectrometer.set_integration_time_micros(request.integration_time_ms * 1000)

        # if request.boxcar_width != 0:
        #     self.spectrometer.f.spectrum_processing.set_boxcar_width(request.boxcar_width)

        scans_to_average = max(request.scans_to_average, 1)
        # broken, so we're just doing this manually
        # if request.scans_to_average != 0:
        #     self.spectrometer.f.spectrum_processing.set_scans_to_average(request.scans_to_average)

        wavelengths: NDArray[numpy.float64] = self.spectrometer.wavelengths().astype(numpy.float64)

        scan_start = time.time()

        intensities_list: list[NDArray[numpy.float64]] = []

        # measure intensities
        for i in range(1, scans_to_average + 1):
            measurement_start = time.time()

            self.get_logger().info(f"Performing measurement {i}")

            intensities: NDArray[numpy.float64] = self.spectrometer.intensities(
                correct_dark_counts=request.correct_dark_counts,
                correct_nonlinearity=request.correct_nonlinearity,
            ).astype(numpy.float64)

            measurement_end = time.time()

            self.get_logger().info(f"Measurement {i} took {(measurement_end - measurement_start):.2f} ms")

            intensities_list.append(intensities)

            feedback = ScienceSpectrometerScan.Feedback()

            feedback.wavelengths = intensities
            feedback.raw_intensities = intensities
            goal_handle.publish_feedback(feedback)

        scan_end = time.time()

        self.get_logger().info(f"Scan took {(scan_end - scan_start) * 1000:.2f} ms")

        temperatures = self.spectrometer.f.temperature.temperature_get_all()

        # TODO 2026-05-10 (Will Free): get the nonlinearity coefficients from the spectrometer and just record them for later.
        #  do not run with this.
        # self.spectrometer.f.nonlinearity_coefficients.get_nonlinearity_coefficients()

        result = ScienceSpectrometerScan.Result()

        result.wavelengths = array.array("d", wavelengths)
        result.intensities = array.array("d", numpy.mean(numpy.array(intensities_list), axis=0))
        result.temperatures = array.array("d", temperatures)

        goal_handle.succeed()
        return result

    @property
    def spectrometer(self) -> Spectrometer:
        return self.__spectrometer


def main():
    try:
        rclpy.init()

        node = SpectrometerService()
        rclpy.spin(node)
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
