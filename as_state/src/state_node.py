#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Bool

class StateMachine(Node):

    def __init__(self):
        super().__init__('state')
        self.subscription_go = self.create_subscription(Bool, 'ebs', self.ebs_callback, 10)
        self.subscription_mission = self.create_subscription(Bool, 'ts_active', self.ts_active_callback, 10)
        self.subscription_ebs = self.create_subscription(Bool, 'r2d', self.r2d_callback, 10)
        self.subscription_odometry = self.create_subscription(Bool, 'brake',self.brake_callback, 10)
        self.subscription_odometry = self.create_subscription(Bool, 'sdc',self.sdc_callback, 10)
        self.subscription_odometry = self.create_subscription(Bool, 'MF',self.MF_callback, 10)
        self.publisher_state = self.create_publisher(String, 'state', 10)
        self.ebs = False
        self.r2d = False
        self.brake = False
        self.sdc = False
        self.MF = False
        self.ts_active = False
        self.state = "AS_Off"
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def ebs_callback(self, msg):
        self.ebs = msg.data    
    def ts_active_callback(self,msg):
        self.ts_active = msg.data
    def r2d_callback(self,msg):
        self.r2d = msg.data
    def brake_callback(self,msg):
        self.brake = msg.data
    def sdc_callback(self,msg):
        self.sdc = msg.data
    def MF_callback(self, msg):
        self.MF = msg.data
    def timer_callback(self):
        msg = String()
    # Bloco para o carro no estado 'AS_Off'
        if self.state == "AS_Off":
            if self.ebs:  # Se EBS ativado
                if self.MF:
                    self.state = "AS_Emergency" if self.sdc else "AS_Finished"
                else:
                    self.state = "AS_Emergency"
            elif self.ts_active:  # Se a missão foi selecionada
                if self.r2d:
                    self.state = "AS_Driving"
                elif self.brake:
                    self.state = "AS_Ready"
                else:
                    self.state = "AS_Off"

    # Bloco para o carro no estado 'AS_Ready'
        if self.state == "AS_Ready":
            if self.ebs:  # Se EBS ativado, carro entra em emergência
                if self.MF:
                    self.state = "AS_Emergency" if self.sdc else "AS_Finished"
                else:
                    self.state = "AS_Emergency"
            elif self.r2d:  # Se pronto para dirigir, muda para Driving
                self.state = "AS_Driving"
            elif not self.brake or not self.ts_active:  # Se brake solto ou missão desativada, desliga
                self.state = "AS_Off"


    # Bloco para o carro no estado 'AS_Driving'
        if self.state == "AS_Driving":
            if self.ebs:  
                if self.MF:
                    self.state = "AS_Emergency" if self.sdc else "AS_Finished"
                else:
                    self.state = "AS_Emergency"
            elif not self.ts_active:
                self.state = "AS_Off"
            elif not self.r2d:
                if self.brake:
                    self.state = "AS_Ready"
                else:
                    self.state = "AS_Off"

    # Bloco para o carro no estado 'AS_Emergency'
        if self.state == "AS_Emergency":
            if self.MF:
                if not self.sdc:
                    self.state = "AS_Finished"
            elif not self.ebs:
                if not self.ts_active:
                    self.state = "AS_Off"
                else:
                    if self.r2d:
                        self.state = "AS_Driving"
                    else:
                        if self.brake:
                            self.state = "AS_Ready"
                        else:
                            self.state = "AS_Off"
        
        # Bloco para o carro no esttado 'AS_Finished'
        if self.state == "AS_Finished":
            if self.sdc:
                self.state = "AS_Emergency"

        msg.data = self.state
        self.publisher_state.publish(msg)
        self.get_logger().info(f'Estado atualizado: {msg.data}')
            
def main(args=None):
    rclpy.init(args=args)

    publisher = StateMachine()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()