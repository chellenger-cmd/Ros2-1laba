#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SmartSafeMapper(Node):
    def __init__(self):
        super().__init__('smart_safe_mapper')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.lidar_data = None
        
        # БЕЗОПАСНЫЕ ПАРАМЕТРЫ
        self.safe_distance = 0.7  # метр до стены - начинаем поворачивать
        self.min_distance = 0.3   # абсолютный минимум - стоп и задний ход
        self.forward_speed = 0.12 # медленная скорость
        self.turn_speed = 0.8     # скорость поворота
        self.backup_speed = -0.1  # скорость заднего хода
        self.backup_time = 0.0    # время движения назад
        
        self.state = "EXPLORING"  # EXPLORING, BACKING_UP, TURNING
        
    def lidar_callback(self, msg):
        self.lidar_data = msg
    
    def get_safe_directions(self):
        if self.lidar_data is None:
            return None
            
        ranges = list(self.lidar_data.ranges)
        num_ranges = len(ranges)
        
        # ДЛЯ ЛИДАРА 360 ГРАДУСОВ:
        # 0° - спереди, 90° - слева, 180° - сзади, 270° - справа
        
        # Спереди: -30° до +30° (330°-30°)
        front_indices = list(range(0, 30)) + list(range(330, 360))
        
        # Слева: 60° до 120° (левый бок)
        left_indices = list(range(60, 120))
        
        # Справа: 240° до 300° (правый бок)  
        right_indices = list(range(240, 300))
        
        # Корректируем индексы для длины массива
        front_indices = [i % num_ranges for i in front_indices]
        left_indices = [i % num_ranges for i in left_indices]
        right_indices = [i % num_ranges for i in right_indices]
        
        # Вычисляем минимальные расстояния
        def get_sector_min(indices):
            valid_ranges = []
            for idx in indices:
                if idx < len(ranges):
                    r = ranges[idx]
                    if not math.isnan(r) and r > 0.1 and r < self.lidar_data.range_max:
                        valid_ranges.append(r)
            return min(valid_ranges) if valid_ranges else self.lidar_data.range_max
        
        front_min = get_sector_min(front_indices)
        left_min = get_sector_min(left_indices)  
        right_min = get_sector_min(right_indices)
        
        # ПРОВЕРКА: действительно ли с одной стороны больше места?
        # Если разница меньше 0.3м - считаем что пространство одинаковое
        left_better = (left_min - right_min) > 0.3
        right_better = (right_min - left_min) > 0.3
        
        return {
            'front': front_min,
            'left': left_min,
            'right': right_min,
            'left_better': left_better,
            'right_better': right_better,
            'equal': not left_better and not right_better
        }
    
    def control_loop(self):
        if self.lidar_data is None:
            return
            
        sectors = self.get_safe_directions()
        if sectors is None:
            return
            
        cmd_vel = Twist()
        
        front_distance = sectors['front']
        left_distance = sectors['left']
        right_distance = sectors['right']
        
        self.get_logger().info(f'State: {self.state} | Front: {front_distance:.2f}, Left: {left_distance:.2f}, Right: {right_distance:.2f}')
        
        # АВТОМАТ КОНЕЧНЫХ СОСТОЯНИЙ
        if self.state == "EXPLORING":
            # ОПАСНО БЛИЗКО - начинаем задний ход
            if front_distance < self.min_distance:
                self.state = "BACKING_UP"
                self.backup_time = 0.0
                cmd_vel.linear.x = self.backup_speed
                cmd_vel.angular.z = 0.0
                self.get_logger().warning('TOO CLOSE! BACKING UP')
                
            # БЛИЗКО К СТЕНЕ - плавно поворачиваем
            elif front_distance < self.safe_distance:
                cmd_vel.linear.x = self.forward_speed * 0.3
                
                # ПРАВИЛЬНЫЙ ВЫБОР СТОРОНЫ ПОВОРОТА
                if sectors['left_better']:
                    cmd_vel.angular.z = self.turn_speed  
                    self.get_logger().info('Turning LEFT - more space on left')
                elif sectors['right_better']:
                    cmd_vel.angular.z = -self.turn_speed
                    self.get_logger().info('Turning RIGHT - more space on right')
                else:
                    # Если пространство одинаковое - поворачиваем в случайную сторону
                    import random
                    cmd_vel.angular.z = self.turn_speed if random.random() > 0.5 else -self.turn_speed
                    self.get_logger().info('Turning RANDOM - equal space')
                    
            else:
                cmd_vel.linear.x = self.forward_speed
                cmd_vel.angular.z = 0.0
                
        elif self.state == "BACKING_UP":
            self.backup_time += 0.1
            cmd_vel.linear.x = self.backup_speed
            cmd_vel.angular.z = 0.0
            
            if self.backup_time >= 1.5:  # 1.5 секунды назад если потерялись
                self.state = "TURNING"
                self.get_logger().info('Backup complete, starting turn')
                
        elif self.state == "TURNING":
            self.backup_time += 0.1
            cmd_vel.linear.x = 0.0
            
            # Выбираем направление поворота на основе реальных данных
            if sectors['left_better']:
                cmd_vel.angular.z = self.turn_speed  # поворот НАЛЕВО
            elif sectors['right_better']:
                cmd_vel.angular.z = -self.turn_speed  # поворот НАПРАВО
            else:
                import random
                cmd_vel.angular.z = self.turn_speed if random.random() > 0.5 else -self.turn_speed
            
            if self.backup_time >= 3.5:
                self.state = "EXPLORING"
                self.get_logger().info('Turn complete, resuming exploration')
        
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SmartSafeMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('Mapper stopped safely')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
