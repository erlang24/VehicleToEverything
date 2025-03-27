#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can

class PcanNode(Node):
    def __init__(self):
        super().__init__('pcan_node')

        # 订阅主题 "/scenes"，收到消息后调用 emergency_callback 回调函数
        self.subscription = self.create_subscription(
            String,
            '/scenes',
            self.emergency_callback,
            10)
        
        # 初始化PCAN总线
        self.bus = None
        self.initialize_pcan()

    def initialize_pcan(self):
        try:
            # 初始化PCAN接口
            self.bus = can.interface.Bus(
                bustype='pcan', 
                channel='PCAN_USBBUS1', 
                bitrate=500000
            )
            self.get_logger().info('PCAN节点已启动')
        except can.CanError:
            self.get_logger().error('PCAN初始化失败')
            self.bus = None

    def emergency_callback(self, msg):
        # 判断是否收到"前项碰撞预警"消息
        if msg.data == "前项碰撞预警" and self.bus:
            self.get_logger().info('收到前向碰撞预警, 发送速度为0的CAN消息')
            
            # 构造一个速度为0的CAN消息（根据您提供的协议）
            speed_zero_msg = can.Message(
                arbitration_id=0x181,  # 与原始测试程序一致
                data=[0x01, 0x5e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],  # 速度为0的数据
                is_extended_id=False
            )
            
            # 发送CAN消息
            self.send_can_message(speed_zero_msg)

    def send_can_message(self, msg):
        """发送CAN消息并处理错误"""
        if self.bus:
            try:
                self.bus.send(msg)
                self.get_logger().info('成功发送CAN消息: %s' % msg)
            except can.CanError:
                self.get_logger().error('发送CAN消息失败')

    def shutdown(self):
        # 关闭PCAN总线
        if self.bus:
            self.bus.shutdown()

def main(args=None):
    rclpy.init(args=args)
    pcan_node = PcanNode()
    
    try:
        rclpy.spin(pcan_node)
    except KeyboardInterrupt:
        pass
    finally:
        pcan_node.shutdown()
        pcan_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
