from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.estimation.d1_estimator import D1Estimator
from rclpy.executors import SingleThreadedExecutor

def main(args: list | None = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, D1Estimator, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
