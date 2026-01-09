#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import time

class MapperMonitor(Node):
    def __init__(self):
        super().__init__('mapper_monitor')

        self.node_name = 'mapper_node'

        self.get_state_client = self.create_client(GetState, f'/{self.node_name}/get_state')
        self.change_state_client = self.create_client(ChangeState, f'/{self.node_name}/change_state')

        self.get_logger().info(f'Aguardando serviços de {self.node_name}...')
        self.get_state_client.wait_for_service()
        self.change_state_client.wait_for_service()

        self.create_timer(5.0, self.monitor_callback)

    def monitor_callback(self):
        future = self.get_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Erro ao consultar estado de {self.node_name}')
            return

        state_id = future.result().current_state.id

        if state_id != 3:  # ACTIVE
            self.get_logger().warn(f'{self.node_name} não está ativo (estado {state_id}). Reiniciando...')
            self.recover_node()
        else:
            self.get_logger().info(f'{self.node_name} está ativo.')

    def recover_node(self):
        self.apply_transition(Transition.TRANSITION_CLEANUP)
        time.sleep(1)
        self.apply_transition(Transition.TRANSITION_CONFIGURE)
        time.sleep(1)
        self.apply_transition(Transition.TRANSITION_ACTIVATE)

    def apply_transition(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done() and future.result().success:
            self.get_logger().info(f'Transição {transition_id} aplicada com sucesso.')
        else:
            self.get_logger().error(f'Erro ao aplicar transição {transition_id}.')

def main(args=None):
    rclpy.init(args=args)
    monitor = MapperMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
