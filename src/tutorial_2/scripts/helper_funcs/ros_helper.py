#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from rclpy.client import Client, SrvTypeRequest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, multiprocessing
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
import rclpy
from rclpy.duration import Duration

def generic_service_call(node:Node, client: Client, client_req, client_res, name_of_srv: str, timeout:float=0.5, times_failed=0, res="success"):
  while not client.wait_for_service(1.0):
    node.get_logger().info(f"{name_of_srv} service not available, waiting again...")
    return generic_service_call(node, client, client_req, client_res, name_of_srv, timeout)
  
  node.get_logger().info(f"Sending {name_of_srv} signal")
  future = client.call_async(client_req)
  rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
  
  if future.result() is not None:
    result : client_res = future.result()
    print(result,"\n\n")
    if result.success:
      client_res.success = result.success
      node.get_logger().info(f"{name_of_srv} Completed")
    return client_res
  else:
    node.get_logger().error("exception while")
  
def generic_service_timer(
  node:Node, 
  client: Client, 
  client_req: SrvTypeRequest, 
  name_of_srv:str, 
  timer_time:float, 
  timeout:float=0.5
  ):
  while not client.wait_for_service(1.0):
    node.get_logger().info(f"{name_of_srv} service not available, waiting again...")
    return generic_service_timer(node, client, client_req, name_of_srv, timer_time, timeout)
  result = None
  async def call_client():
    nonlocal result
    try:
      future = client.call_async(client_req)
      result = await future
    except Exception:
      pass
  return node.create_timer(1, call_client, callback_group=ReentrantCallbackGroup()), result



def wait_for_message(node:Node, MsgType, topic_name, time_to_wait:int = -1):
    future = Future()
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )
    def callback(msg):
      future.set_result(msg)

    subscription = node.create_subscription(MsgType, topic_name, callback, qos_profile)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if time_to_wait != -1:
      end_time = node.get_clock().now() + Duration(seconds=time_to_wait)
      while node.get_clock().now() < end_time:
        if future.done():
          return future.result()
        executor.spin_once(timeout_sec=max(0.01, (end_time - node.get_clock().now()).nanoseconds / 1e9))
      return None
    else:
      while not future.done():
        executor.spin_once(timeout_sec=0.01)
      return future.result()