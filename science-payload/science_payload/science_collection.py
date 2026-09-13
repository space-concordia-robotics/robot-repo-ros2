# ruff: noqa: D100, D101, D102, D103, D107
from enum import Enum
from pathlib import Path

import numpy
import numpy as np
import pandas as pd
import rclpy
from numpy.typing import NDArray
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rover_msgs.action import ScienceSpectrometerScan


class SampleType(Enum):
    BASELINE = "baseline"
    PH = "ph"
    RESAZURIN = "resazurin"


class ScienceCollection(Node):
    __site: int
    __cuvette: int
    __sample: SampleType
    __characteristic: str | None
    __light_pollution: bool

    __send_goal_future: Future | None
    __get_result_future: Future | None
    __action_client: ActionClient

    __background: NDArray[numpy.float64]

    __feedback_received: int

    def __init__(self):
        super().__init__("science_collection")

        self.__site = self.declare_parameter("site", Parameter.Type.INTEGER).get_parameter_value().integer_value
        self.__cuvette = self.declare_parameter("cuvette", Parameter.Type.INTEGER).get_parameter_value().integer_value
        sample = self.declare_parameter("sample", Parameter.Type.STRING).get_parameter_value().string_value
        if sample not in [t.value for t in SampleType]:
            self.get_logger().error(f"parameter sample must be one of: {', '.join(t.value for t in SampleType)}, but was: {sample}")
            rclpy.shutdown()
            return
        self.__sample = SampleType[sample.upper()]
        self.__characteristic = self.declare_parameter("characteristic", "").get_parameter_value().string_value
        self.__characteristic = None if self.__characteristic == "" else self.__characteristic
        self.__light_pollution = self.declare_parameter("light_pollution", False).get_parameter_value().bool_value

        self.__send_goal_future = None
        self.__get_result_future = None
        self.__action_client = ActionClient(
            self,
            ScienceSpectrometerScan,
            "/rover/science/scan",
        )

        self.__feedback_received = 0

        self.send_goal()

    @property
    def site(self) -> int:
        return self.__site

    @property
    def cuvette(self) -> int:
        return self.__cuvette

    @property
    def sample(self) -> SampleType:
        return self.__sample

    @property
    def characteristic(self) -> str | None:
        return self.__characteristic

    @property
    def light_pollution(self) -> bool:
        return self.__light_pollution

    @property
    def feedback_received(self) -> int:
        return self.__feedback_received

    def send_goal(self):
        goal = ScienceSpectrometerScan.Goal()

        goal.scans_to_average = 10
        goal.boxcar_width = 5
        goal.integration_time_ms = 20

        self.__action_client.wait_for_server()

        self.__send_goal_future = self.__action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        assert self.__send_goal_future is not None
        self.__send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):  # noqa: ANN001
        self.__feedback_received += 1
        feedback = feedback.feedback
        intensities: NDArray[numpy.float64] = numpy.array(feedback.raw_intensities, dtype=numpy.float64)
        self.save_data(intensities, f"raw/raw-intensities-{self.feedback_received}")
        self.get_logger().info("Got raw measurement from spectrometer.")

    def goal_response_callback(self, future):  # noqa: ANN001
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("spectrometer service rejected goal")
            return

        self.__get_result_future = goal_handle.get_result_async()
        assert self.__get_result_future is not None
        self.__get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):  # noqa: ANN001
        result: ScienceSpectrometerScan.Result = future.result().result

        wavelengths: NDArray[numpy.float64] = numpy.array(result.wavelengths, dtype=numpy.float64)
        intensities: NDArray[numpy.float64] = numpy.array(result.intensities, dtype=numpy.float64)

        self.save_data(wavelengths, "wavelengths")
        self.save_data(intensities, "intensities")

        if self.sample == SampleType.BASELINE:
            rclpy.shutdown()
            return

        baseline = self.load_baseline("intensities")

        if baseline is None:
            # could not load baseline, already logged error.
            rclpy.shutdown()
            return

        background = self.load_background(self.light_pollution)

        if background is None:
            # could not load background, already logged error.
            rclpy.shutdown()
            return

        self.save_data(
            ScienceCollection.intensity_to_absorbance(intensities, background),
            "intensities-absorbance",
        )

        intensity_no_baseline = numpy.subtract(intensities, baseline)
        intensity_no_background = numpy.subtract(intensities, background)

        self.save_data(intensity_no_baseline, "no-baseline")
        self.save_data(intensity_no_background, "no-background")

        self.save_data(
            ScienceCollection.intensity_to_absorbance(intensity_no_baseline, background),
            "no-baseline-absorbance",
        )
        self.save_data(
            ScienceCollection.intensity_to_absorbance(intensity_no_background, background),
            "no-background-absorbance",
        )

        rclpy.shutdown()

    def save_data(self, data: NDArray[numpy.float64], prefix: str):
        dataframe = pd.DataFrame(data)
        filename = ScienceCollection.filename(prefix, "csv", self.site, self.cuvette, self.sample, self.characteristic)
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Saving data to {filename}")
        dataframe.to_csv(filename)

    def load_baseline(self, prefix: str) -> NDArray[numpy.float64] | None:
        filename = ScienceCollection.filename(prefix, "csv", self.site, self.cuvette, SampleType.BASELINE, self.characteristic)

        if not Path(filename).exists():
            self.get_logger().info(f"Cannot load data from {filename}, did you measure the baseline?")
            return None

        self.get_logger().info(f"Loading baseline from {filename}")

        tmp = pd.read_csv(filename, index_col=0).to_numpy(dtype=numpy.float64)
        return tmp.reshape((tmp.shape[0],))

    def load_background(self, light_pollution: bool) -> NDArray[numpy.float64] | None:
        filename = "StrayLightLidOff.csv" if light_pollution else "StrayLightLidOn.csv"

        if not Path(filename).exists():
            self.get_logger().info(f"Cannot load data from {filename}, are the background values in the right place?")
            return None

        self.get_logger().info(f"Loading background from {filename}")

        tmp: NDArray[numpy.float64] = pd.read_csv(filename, header=None, index_col=False, usecols=[1]).to_numpy(dtype=numpy.float64)
        return tmp.reshape((tmp.shape[0],))

    @staticmethod
    def remove_baseline(data: NDArray[numpy.float64], baseline: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        assert data.size == baseline.size

        return data - baseline

    @staticmethod
    def intensity_to_absorbance(data: NDArray[numpy.float64], background: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        return numpy.log10(np.maximum(0, data / background))

    @staticmethod
    def filename(prefix: str, ext: str, site: int, cuvette: int, sample: SampleType, characteristic: str | None) -> str:
        if characteristic is not None:
            return f"site-{site}/{prefix}.cu-{cuvette}.{sample.value}.{characteristic}.{ext}"
        return f"site-{site}/{prefix}.cu-{cuvette}.{sample.value}.{ext}"


def main():
    try:
        rclpy.init()

        node = ScienceCollection()
        rclpy.spin(node)
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
