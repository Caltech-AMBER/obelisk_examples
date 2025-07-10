from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from d1_control.Controller import Controller

def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    print("Starting D1 Example Controller...")
    spin_obelisk(args, Controller, SingleThreadedExecutor)

if __name__ == "__main__":
    main()