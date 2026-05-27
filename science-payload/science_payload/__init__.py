import rclpy
from rclpy.executors import ExternalShutdownException

from .science_payload import SciencePayload


def main(args=None):
    try:
        rclpy.init(args=args)

        node = SciencePayload()
        rclpy.spin(node)
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
