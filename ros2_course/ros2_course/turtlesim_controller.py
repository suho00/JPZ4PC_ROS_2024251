import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
import numpy as np


class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        # ROS param declarations
        self.declare_parameter('speed', 6.0)
        self.declare_parameter('omega', 10.0)

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


    # Go to method
    def go_to(self, target_x, target_y):
        # 1. Várakozás, amíg a teknős "felébred" (megjön az első pozíció)
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Várakozás a pozícióra...')
            rclpy.spin_once(self)

        # 2. Beállítások (Ezek a P-szabályozó "erősítései")
        Kp_linear = 1.0   # Mennyire siessen a cél felé?
        Kp_angular = 4.0  # Mennyire agresszívan forduljon?

        distance_tolerance = 0.1 # Ha ennyire közel ér, akkor "ott vagyunk"

        # 3. A Ciklus (Closed Loop Control)
        # Addig futunk, amíg a távolság nagyobb, mint a tűréshatár
        distance = 100.0 # Kezdeti kamu érték, hogy belépjen a ciklusba

        loop_rate = self.create_rate(20) # 20 Hz (másodpercenként 20-szor számol)

        while rclpy.ok():
            # Hol vagyunk most?
            current_x = self.pose.x
            current_y = self.pose.y
            current_theta = self.pose.theta

            # Távolság számítása (Pitagorasz tétel)
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx**2 + dy**2)

            # Ha elég közel vagyunk, lépjünk ki!
            if distance < distance_tolerance:
                break

            # Szög számítása (Milyen irányba kell menni?)
            desired_theta = math.atan2(dy, dx)
            angular_error = desired_theta - current_theta

            # SZÖG NORMALIZÁLÁS (Trükkös rész!)
            # Ha pl. 350 fokot kellene fordulni balra, inkább forduljon 10 fokot jobbra.
            if angular_error > math.pi:
                angular_error -= 2 * math.pi
            elif angular_error < -math.pi:
                angular_error += 2 * math.pi

            # --- ITT VAN A LÉNYEG: P-SZABÁLYOZÓ ---
            cmd = Twist()

            # A sebesség arányos a hibával (távolsággal)
            # De korlátozzuk le max 2.0-ra, nehogy túlgyorsuljon
            linear_speed = Kp_linear * distance
            cmd.linear.x = min(linear_speed, 2.0)

            # A forgás arányos a szögeltéréssel
            cmd.angular.z = Kp_angular * angular_error

                # Parancs küldése
            self.twist_pub.publish(cmd)

            # Frissítés
            rclpy.spin_once(self)
            loop_rate.sleep()

        # 4. Megérkeztünk -> Állj meg!
        stop_cmd = Twist()
        self.twist_pub.publish(stop_cmd)
        self.get_logger().info(f'Megérkeztem: ({target_x}, {target_y})')

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
    #tc.go_straight(1.0, 4.0)
    #tc.turn(90.0, 90.0)
    #tc.go_straight(1.0, 4.0)
    #tc.draw_square(speed=1.0, omega=90.0, a=3)
    #tc.draw_poly(speed=1.0, omega=90.0, N=8, a=1)
    tc.setup_drawing(x=1.0, y=7.0, theta=0.0)

    tc.draw_snowflake(length=6.0, order=3)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
