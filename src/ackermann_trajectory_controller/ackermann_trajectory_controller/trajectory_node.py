import csv
import math
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AckermannTrajectoryController(Node):
    """Pure Pursuit para UN ocho de Gerono: cierre en origen y parada automatica."""

    def __init__(self):
        super().__init__('ackermann_trajectory_controller')
        # Parámetros ajustables por línea de comandos ROS 2.
        self.declare_parameter('wheelbase', 0.24)
        self.declare_parameter('speed', 0.40)
        self.declare_parameter('lookahead', 0.65)
        self.declare_parameter('max_steer', 0.48)
        self.declare_parameter('radius', 2.1)
        # AckermannSteering inicia /odom en yaw=0 incluso si el robot aparece rotado en world.
        # Rotar la referencia -45 grados la alinea con el eje X de la odometria.
        self.declare_parameter('path_rotation_deg', -45.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('slowdown_distance', 0.60)
        self.declare_parameter('csv_path', os.path.join(os.path.expanduser('~'), 'resultados_trayectoria.csv'))

        self.L = float(self.get_parameter('wheelbase').value)
        self.v_ref = float(self.get_parameter('speed').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.radius = float(self.get_parameter('radius').value)
        self.path_rotation_deg = float(self.get_parameter('path_rotation_deg').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        if (self.L <= 0.0 or self.lookahead <= 0.0 or self.radius <= 0.0
                or self.max_steer <= 0.0 or self.goal_tolerance <= 0.0
                or self.slowdown_distance <= self.goal_tolerance):
            raise ValueError('Parámetros geométricos no válidos')

        # Todos los waypoints y sus errores quedan en el marco 'odom'.
        raw_waypoints = self.generate_figure_eight(self.radius, 200)
        beta = math.radians(self.path_rotation_deg)
        c, s = math.cos(beta), math.sin(beta)
        self.waypoints = [(c*x - s*y, s*x + c*y) for x, y in raw_waypoints]
        self.current_idx = 0
        self.is_running = True
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.initial_heading_checked = False
        self.log_data = []
        self.csv_saved = False
        self.start_time = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Ocho Ackermann | radio parametrico={self.radius:.2f} m | '
            f'lookahead={self.lookahead:.2f} m | '
            f'rotacion_referencia={self.path_rotation_deg:.1f} grados | '
            f'fuente_pose={self.odom_topic}. '
            'Usa --ros-args -p use_sim_time:=true al ejecutar el nodo.'
        )

    @staticmethod
    def generate_figure_eight(radius, num_points):
        return [
            (radius * math.sin(2.0 * math.pi * i / num_points),
             radius * math.sin(2.0 * math.pi * i / num_points) * math.cos(2.0 * math.pi * i / num_points))
            for i in range(num_points + 1)
        ]

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_received = True

    def find_lookahead_point(self):
        """Busca hacia adelante SIN modulo: la ruta acaba en el punto final."""
        n = len(self.waypoints)
        for idx in range(self.current_idx, n):
            wx, wy = self.waypoints[idx]
            dist = math.hypot(wx - self.x, wy - self.y)
            if dist >= self.lookahead:
                self.current_idx = idx
                return wx, wy, dist

        # Ultimo tramo: perseguir el origen sin volver al primer waypoint.
        self.current_idx = n - 1
        wx, wy = self.waypoints[-1]
        return wx, wy, math.hypot(wx - self.x, wy - self.y)

    def nearest_path_projection(self):
        """Proyección sobre la polilínea real, NO el punto de anticipación."""
        best_sq = float('inf')
        best_x = best_y = 0.0
        n = len(self.waypoints)
        for i in range(n):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[(i + 1) % n]
            vx, vy = bx - ax, by - ay
            denom = vx * vx + vy * vy
            if denom < 1e-12:
                continue
            t = max(0.0, min(1.0, ((self.x - ax) * vx + (self.y - ay) * vy) / denom))
            px, py = ax + t * vx, ay + t * vy
            d_sq = (self.x - px) ** 2 + (self.y - py) ** 2
            if d_sq < best_sq:
                best_sq = d_sq
                best_x, best_y = px, py
        return best_x, best_y, math.sqrt(best_sq)

    def log_sample(self, delta):
        ref_x, ref_y, error = self.nearest_path_projection()
        target_x, target_y = self.waypoints[self.current_idx]
        now = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.log_data.append([
            round(now, 3), round(self.x, 4), round(self.y, 4),
            round(ref_x, 4), round(ref_y, 4),
            round(target_x, 4), round(target_y, 4),
            round(delta, 4), round(error, 4), round(self.yaw, 4),
        ])

    def finish_route(self, reason):
        """Pone velocidad cero, guarda CSV una sola vez y conserva frenado."""
        if not self.is_running:
            return
        self.is_running = False
        self.cmd_pub.publish(Twist())
        self.get_logger().info(
            f'{reason} | posicion final odom=({self.x:.3f}, {self.y:.3f}) m | '
            f'distancia al origen={math.hypot(self.x, self.y):.3f} m'
        )
        self.save_csv()

    def control_loop(self):
        if not self.odom_received:
            return
        if not self.is_running:
            # Seguir publicando cero por seguridad, incluso despues de llegar.
            self.cmd_pub.publish(Twist())
            return
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        if not self.initial_heading_checked:
            dx = self.waypoints[1][0] - self.waypoints[0][0]
            dy = self.waypoints[1][1] - self.waypoints[0][1]
            expected_heading = math.atan2(dy, dx)
            difference = math.atan2(
                math.sin(self.yaw - expected_heading),
                math.cos(self.yaw - expected_heading),
            )
            if abs(difference) > math.radians(10.0):
                self.get_logger().warn(
                    f'Desalineacion inicial de rumbo: {math.degrees(difference):.1f} grados. '
                    'Revisa path_rotation_deg y el marco /odom.'
                )
            self.initial_heading_checked = True

        target_x, target_y, dist = self.find_lookahead_point()
        goal_x, goal_y = self.waypoints[-1]
        goal_distance = math.hypot(goal_x - self.x, goal_y - self.y)
        near_final_segment = (self.current_idx == len(self.waypoints) - 1)

        # Nunca finalizar en el cruce central: exigimos que el objetivo
        # haya avanzado hasta el ULTIMO waypoint del ocho.
        if near_final_segment and goal_distance <= self.goal_tolerance:
            self.log_sample(0.0)
            self.finish_route('Ocho completado: robot detenido automaticamente')
            return

        angle = math.atan2(target_y - self.y, target_x - self.x)
        alpha = math.atan2(math.sin(angle - self.yaw), math.cos(angle - self.yaw))
        delta = math.atan2(2.0 * self.L * math.sin(alpha), max(dist, 0.2))
        delta = max(-self.max_steer, min(self.max_steer, delta))
        v_cmd = self.v_ref * (1.0 - 0.35 * abs(delta) / self.max_steer)

        # Frenar progresivamente SOLO al aproximarse al final de la ruta.
        if near_final_segment:
            scale = min(1.0, goal_distance / self.slowdown_distance)
            v_cmd = min(v_cmd, max(0.07, self.v_ref * scale))

        omega = (v_cmd / self.L) * math.tan(delta)
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)
        self.log_sample(delta)

    def save_csv(self):
        if self.csv_saved:
            return
        if not self.log_data:
            self.get_logger().warn('Todavía no hay muestras para guardar.')
            return
        directory = os.path.dirname(os.path.abspath(self.csv_path))
        os.makedirs(directory, exist_ok=True)
        with open(self.csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'tiempo_s', 'x_real', 'y_real', 'x_ref', 'y_ref',
                'x_lookahead', 'y_lookahead', 'angulo_direccion_rad',
                'error_seguimiento_m', 'yaw_real_rad',
            ])
            writer.writerows(self.log_data)
        self.csv_saved = True
        self.get_logger().info(f'Guardadas {len(self.log_data)} muestras en {self.csv_path}')

    def emergency_stop(self):
        if rclpy.ok():
            for _ in range(5):
                self.cmd_pub.publish(Twist())
        if self.is_running:
            self.is_running = False
            self.get_logger().info('Interrumpido manualmente antes de completar el ocho.')
        self.save_csv()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannTrajectoryController()

    def handler(sig, frame):
        try:
            node.emergency_stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
