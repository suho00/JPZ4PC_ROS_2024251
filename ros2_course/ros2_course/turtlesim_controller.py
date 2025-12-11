import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen


class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        # ROS param declarations
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('omega', 90.0)

        self.twist_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

    # New method for TurtlesimController
    def cb_pose(self, msg):
        self.pose = msg
        self.get_logger().info(f'Pose frissült: x={msg.x:.2f}, y={msg.y:.2f}, th={msg.theta:.2f}')

        #self.get_logger().info('Pose:' + str(self.pose))

    def go_straight(self, distance):
        # Ros param
        speed = self.get_parameter('speed').get_parameter_value().double_value

        # Create and publish msg
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(
            100,
            self.get_clock()) # Hz

        # Calculate time
        T = abs(distance / speed)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when and rclpy.ok()):
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')


    def turn(self, angle):
        # ROS param
        omega = self.get_parameter('omega').get_parameter_value().double_value

        # Implement rotation here
        # Create and publish msg
        omega_rad = math.radians(omega)
        angle_rad = math.radians(angle)

        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle_rad > 0:
            vel_msg.angular.z = omega_rad
        else:
            vel_msg.angular.z = -omega_rad

        # Set loop rate
        loop_rate = self.create_rate(
            100,
            self.get_clock()) # Hz

        # Calculate time
        T = abs(angle_rad / omega_rad)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started turning.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when and rclpy.ok()):
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('Turning...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turned to destination.')


    def draw_square(self, speed, omega, a):
        for i in range(4):
            self.go_straight(a)
            self.turn(90.0)

    def draw_poly(self, speed, omega, N, a):
        angle = 360.0 / N
        for i in range(N):
            self.go_straight(a)
            self.turn(angle)


    def go_to(self, target_x, target_y):
        # 1. Várjuk meg, amíg megjön az első pozíció
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Várakozás a pozícióra... (pose még None)')
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.pose is None:
            self.get_logger().error('Nincs pose, nem tudok szabályozni.')
            return

        self.get_logger().info(
            f'go_to indult célra: x={target_x:.2f}, y={target_y:.2f}, '
            f'aktuális: x={self.pose.x:.2f}, y={self.pose.y:.2f}, th={self.pose.theta:.2f}'
        )

        # --- P-szabályozó paraméterek ---
        Kp_linear = 0.8
        Kp_angular = 2.0

        distance_tolerance = 0.1
        max_linear = 2.0
        max_angular = 2.0  # rad/s

        # Maximum iteráció, hogy végtelen ciklust biztosan elkerüljük
        for i in range(500):  # 500 * 0.05s ≈ 25 másodperc
            if not rclpy.ok():
                break

            if self.pose is None:
                self.get_logger().warn('Pose None a ciklus közben!')
                break

            x = self.pose.x
            y = self.pose.y
            theta = self.pose.theta

            # Szög normalizálás -pi..pi közé
            while theta > math.pi:
                theta -= 2 * math.pi
            while theta < -math.pi:
                theta += 2 * math.pi

            dx = target_x - x
            dy = target_y - y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < distance_tolerance:
                self.get_logger().info(
                    f'Kilépés: distance < tol ({distance:.3f} < {distance_tolerance})'
                )
                break

            desired_theta = math.atan2(dy, dx)
            angular_error = desired_theta - theta

            # Szöghiba normalizálása
            if angular_error > math.pi:
                angular_error -= 2 * math.pi
            elif angular_error < -math.pi:
                angular_error += 2 * math.pi

            cmd = Twist()

            # Ha nagyon rossz irányba néz, inkább csak forduljon
            if abs(angular_error) > 1.0:  # ~57°
                cmd.linear.x = 0.0
            else:
                v = Kp_linear * distance
                cmd.linear.x = min(v, max_linear)

            w = Kp_angular * angular_error
            if w > max_angular:
                w = max_angular
            elif w < -max_angular:
                w = -max_angular
            cmd.angular.z = w

            if i % 10 == 0:
                self.get_logger().info(
                    f'[go_to i={i}] pos=({x:.2f},{y:.2f}), dist={distance:.3f}, '
                    f'ang_err={angular_error:.3f}, v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}'
                )

            self.twist_pub.publish(cmd)

            # Itt várunk egy kicsit, közben érkezik új pose
            rclpy.spin_once(self, timeout_sec=0.05)

        # Megállás
        self.twist_pub.publish(Twist())
        if self.pose is not None:
            self.get_logger().info(
                f'go_to vége. Végső pozíció: x={self.pose.x:.2f}, y={self.pose.y:.2f}, th={self.pose.theta:.2f}'
            )
        else:
            self.get_logger().info('go_to vége. Pose None.')


        # --- ÚJ: SEGÉD METÓDUSOK A RAJZOLÁSHOZ ---

    def setup_drawing(self, x, y, theta):
        """Teleportálja a teknőst a kezdőpozícióba rajzolás nélkül."""
        # Kliens létrehozása a teleportáláshoz
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Várakozás a teleport service-re...')

        # Pen (toll) kikapcsolása, hogy ne húzzon csíkot teleport közben
        pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        if pen_client.wait_for_service(timeout_sec=1.0):
            req_pen = SetPen.Request()
            req_pen.off = 1  # Toll fel (nem rajzol)
            pen_client.call_async(req_pen)

        # Teleport kérése
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        future = teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Toll visszakapcsolása
        if pen_client.wait_for_service(timeout_sec=1.0):
            req_pen = SetPen.Request()
            req_pen.off = 0  # Toll le (rajzol)
            pen_client.call_async(req_pen)

        self.get_logger().info(f'Készen állok a rajzolásra itt: {x}, {y}')


    # --- A FRAKTÁL LOGIKA (REKURZIÓ) ---

    def koch_curve(self, length, order):
        """
        Rekurzív függvény egyetlen Koch-görbe szakasz megrajzolásához.
        length: a szakasz hossza
        order: a rekurzió mélysége (minél nagyobb, annál részletesebb)
        """
        if order == 0:
            # Alapeset: Csak menj előre
            self.go_straight(length)
        else:
            # Rekurzív lépés: F1 -> B60 -> F1 -> J120 -> F1 -> B60 -> F1
            # A hosszat harmadoljuk
            seg_len = length / 3.0

            self.koch_curve(seg_len, order - 1)

            self.turn(60.0)
            self.koch_curve(seg_len, order - 1)

            self.turn(-120.0)
            self.koch_curve(seg_len, order - 1)

            self.turn(60.0)
            self.koch_curve(seg_len, order - 1)

    def draw_snowflake(self, length, order):
        """
        Három Koch-görbe alkot egy hópihét.
        """
        self.get_logger().info(f'Hópihe rajzolása (Order: {order})...')
        for _ in range(3):
            self.koch_curve(length, order)
            self.turn(-120.0) # Fordulás a következő oldalhoz

        self.get_logger().info('Hópihe kész!')

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    tc.setup_drawing(x=8.5, y=5.5, theta=0.0)

    tc.go_to(7.0, 9.0)
    tc.go_to(9.0, 2.0)
    tc.go_to(8.5, 8.0)

    tc.setup_drawing(x=1.0, y=7.0, theta=0.0)
    tc.draw_snowflake(length=6.0, order=3)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
