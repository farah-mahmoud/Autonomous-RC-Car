import rclpy
import rclpy.executors
from .tkio_ros import TkioRos

def main(args=None):
    rclpy.init(args=args)
    exe = rclpy.executors.SingleThreadedExecutor()

    tkio = TkioRos()

    exe.add_node(tkio)

    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tkio.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()