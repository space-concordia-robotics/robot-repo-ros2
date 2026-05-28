import textwrap
import time
import uuid
from time import sleep
from typing import List, Dict

import array
import numpy
import pandas
import rclpy
from numpy.typing import NDArray
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rover_msgs.action import ScienceSpectrometerScan
from seabreeze.spectrometers import Spectrometer, list_devices as list_oceanoptics_devices
from seabreeze.types import SeaBreezeDevice


class SciencePayload(Node):
    __spectrometer: Spectrometer

    def __init__(self):
        super().__init__('science_payload')

        devices: List[SeaBreezeDevice]

        while True:
            devices = list_oceanoptics_devices()

            if len(devices) > 0:
                break
            else:
                self.get_logger().info(f'Waiting for Ocean Optics spectrometer device...')

            sleep(5)

        self.get_logger().info(f'Found {len(devices)} Ocean Optics spectrometer devices')
        if len(devices) != 1:
            self.get_logger().error(f'More than 1 spectrometer found, unsure which devices to use. Exiting.')
            rclpy.shutdown()

        device = devices[0]

        self.__spectrometer = Spectrometer(device)

        features: Dict[str, List[str]] = {feature: [item.identifier for item in feature_items] for feature, feature_items in self.spectrometer.features.items()}

        features_string = {feature: '\n'.join(f'* {item}' for item in items) for feature, items in features.items()}
        features_string = '\n'.join([f'* {feature}\n{textwrap.indent(items, '  ')}' for feature, items in features_string])

        # TODO 2026-05-10 (Will Free): print hardware revision & firmware revision here (in revision feature)

        # TODO 2026-05-10 (Will Free): sigh, multiline strings aren't un-indented automatically so I need to do this garbage.
        #  and it's annoying to have to wrap the entire thing in textwrap.dedent.
        self.get_logger().info(f'''Found spectrometer:
* model: {self.spectrometer.model}
* serial number: {self.spectrometer.serial_number}
* wavelengths: {min(self.spectrometer.wavelengths())} nm - {max(self.spectrometer.wavelengths())} nm
* pixels: {self.spectrometer.pixels}
* integration time hard limits: {self.spectrometer.integration_time_micros_limits[0]} µs - {self.spectrometer.integration_time_micros_limits[1]} µs
* features:
{textwrap.indent(features_string, '  ')}
''')
        self.__scan_action_server = ActionServer(
            self,
            ScienceSpectrometerScan,
            '/rover/science/scan',
            self.execute_scan,
        )

    def execute_scan(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Triggered spectrometer scan')

        # TODO 2026-05-10 (Will Free): add field to request for light sources
        request: ScienceSpectrometerScan.Goal = goal_handle.request

        # TODO 2026-05-12 (Will Free): add code to turn light on before measurement & off afterwards?

        if request.integration_time != 0:
            self.spectrometer.f.spectrometer.set_integration_time_micros(request.integration_time_ms * 1000)

        if request.boxcar_width != 0:
            self.spectrometer.f.spectrum_processing.set_boxcar_width(request.boxcar_width)

        if request.scans_to_average != 0:
            self.spectrometer.f.spectrum_processing.set_scans_to_average(request.scans_to_average)

        wavelengths: NDArray[numpy.float64] = self.spectrometer.wavelengths().astype(numpy.float64)

        scan_start = time.time()

        # measure intensities
        intensities: NDArray[numpy.float64] = self.spectrometer.intensities(
            correct_dark_counts=request.correct_dark_counts,
            correct_nonlinearity=request.correct_nonlinearity
        ).astype(numpy.float64)

        scan_end = time.time()

        spectrum = numpy.vstack(
            (
                wavelengths,
                intensities
            )
        )

        self.get_logger().info(f'Scan took {(scan_start - scan_end) * 1000:.2f} ms')

        temperatures = self.spectrometer.f.temperature.temperature_get_all()

        file_uuid = uuid.uuid4()
        spectrum_filename = f'spectrum-{file_uuid}.csv'
        temperatures_filename = f'temperature-{file_uuid}.csv'

        spectrum_dataframe = pandas.DataFrame(spectrum)
        spectrum_dataframe.to_csv(spectrum_filename)

        temperatures_dataframe = pandas.DataFrame(temperatures)
        temperatures_dataframe.to_csv(temperatures_filename)

        self.get_logger().info(f'Saved spectrum to {spectrum_filename} and temperatures to {temperatures_filename}')

        # TODO 2026-05-10 (Will Free): get the nonlinearity coefficients from the spectrometer and just record them for later. do not run with this.
        # self.spectrometer.f.nonlinearity_coefficients.get_nonlinearity_coefficients()

        result = ScienceSpectrometerScan.Result()

        result.wavelengths = array.array('d', wavelengths)
        result.intensities = array.array('d', intensities)
        result.temperatures = array.array('d', temperatures)

        goal_handle.succeed()
        return result

    @property
    def spectrometer(self) -> Spectrometer:
        return self.__spectrometer
