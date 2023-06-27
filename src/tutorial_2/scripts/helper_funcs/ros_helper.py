#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from rclpy.client import Client, SrvTypeRequest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, multiprocessing

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