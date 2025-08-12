#!/usr/bin/env python3
"""
Elevator Scheduler Node for Multi-Floor Robot Navigation

This node provides high-level coordination and scheduling for multiple elevators,
ensuring optimal availability for robot navigation between floors.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import AddTwoInts  # We'll create a custom service later
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional
from enum import Enum

# For now, we'll use a simple custom message structure
# In a full implementation, you'd create custom ROS2 services

class ElevatorState(Enum):
    IDLE = "IDLE"
    CLOSING_DOORS = "CLOSING_DOORS"
    MOVING_UP = "MOVING_UP"
    MOVING_DOWN = "MOVING_DOWN"
    OPENING_DOORS = "OPENING_DOORS"
    UNKNOWN = "UNKNOWN"

@dataclass
class ElevatorStatus:
    """Tracks the status of a single elevator"""
    elevator_id: int
    current_floor: int = 0
    doors_open: bool = True
    state: ElevatorState = ElevatorState.UNKNOWN
    position: Optional[PoseStamped] = None
    last_update: float = 0.0
    available_for_call: bool = True
    
    def is_available(self) -> bool:
        """Check if elevator is available for robot use"""
        return (self.state == ElevatorState.IDLE and 
                self.doors_open and 
                self.available_for_call)
    
    def is_moving(self) -> bool:
        """Check if elevator is currently moving"""
        return self.state in [ElevatorState.MOVING_UP, ElevatorState.MOVING_DOWN]

class ElevatorScheduler(Node):
    """Main elevator scheduling and coordination node"""
    
    def __init__(self):
        super().__init__('elevator_scheduler')
        
        # Configuration
        self.declare_parameter('num_elevators', 4)
        self.declare_parameter('min_available_elevators', 1)
        self.declare_parameter('max_simultaneous_moving', 2)
        self.declare_parameter('status_update_rate', 2.0)  # Hz
        self.declare_parameter('coordination_enabled', True)
        
        self.num_elevators = self.get_parameter('num_elevators').value
        self.min_available = self.get_parameter('min_available_elevators').value
        self.max_moving = self.get_parameter('max_simultaneous_moving').value
        self.update_rate = self.get_parameter('status_update_rate').value
        self.coordination_enabled = self.get_parameter('coordination_enabled').value
        
        # Elevator tracking
        self.elevators: Dict[int, ElevatorStatus] = {}
        self.lock = threading.Lock()
        
        # Initialize elevator status tracking
        for i in range(1, self.num_elevators + 1):
            self.elevators[i] = ElevatorStatus(elevator_id=i)
        
        # ROS Service for elevator requests
        self.request_service = self.create_service(
            AddTwoInts,
            'request_elevator',
            self.handle_elevator_request
        )
        
        # Group control services
        self.bring_all_service = self.create_service(
            AddTwoInts,
            'bring_all_to_floor',
            self.handle_bring_all_request
        )
        self.get_logger().info('DEBUG: bring_all_to_floor service created successfully')
        
        self.release_all_service = self.create_service(
            AddTwoInts,
            'release_all_elevators',
            self.handle_release_all_request
        )
        
        # Service clients for individual elevator control
        self.elevator_goto_clients = {}
        self.elevator_hold_clients = {}
        self.elevator_release_clients = {}
        
        for elevator_id in range(1, self.num_elevators + 1):
            # Create service clients for each elevator
            self.elevator_goto_clients[elevator_id] = self.create_client(
                AddTwoInts, f'/elevator_{elevator_id}/goto'
            )
            self.elevator_hold_clients[elevator_id] = self.create_client(
                AddTwoInts, f'/elevator_{elevator_id}/hold'
            )
            self.elevator_release_clients[elevator_id] = self.create_client(
                AddTwoInts, f'/elevator_{elevator_id}/release'
            )
        
        # Subscribers for each elevator
        self.floor_subscribers = {}
        self.door_subscribers = {}
        self.position_subscribers = {}
        
        for elevator_id in range(1, self.num_elevators + 1):
            # Subscribe to current floor
            self.floor_subscribers[elevator_id] = self.create_subscription(
                Int32,
                f'/elevator_{elevator_id}/current_floor',
                lambda msg, eid=elevator_id: self.floor_callback(msg, eid),
                10
            )
            
            # Subscribe to door state
            self.door_subscribers[elevator_id] = self.create_subscription(
                Bool,
                f'/elevator_{elevator_id}/door_state',
                lambda msg, eid=elevator_id: self.door_callback(msg, eid),
                10
            )
            
            # Subscribe to position
            self.position_subscribers[elevator_id] = self.create_subscription(
                PoseStamped,
                f'/elevator_{elevator_id}/car_position',
                lambda msg, eid=elevator_id: self.position_callback(msg, eid),
                10
            )
        
        # # Status publishing
        # self.status_timer = self.create_timer(
        #     1.0 / self.update_rate, 
        #     self.publish_status
        # )
        
        # # Coordination timer
        # if self.coordination_enabled:
        #     self.coordination_timer = self.create_timer(
        #         5.0,  # Every 5 seconds
        #         self.coordinate_elevators
        #     )
        
        self.get_logger().info(f'Elevator Scheduler initialized for {self.num_elevators} elevators')
        self.get_logger().info(f'Coordination: {self.coordination_enabled}, Min available: {self.min_available}')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /request_elevator - Request elevator for robot transport')
        self.get_logger().info('  /bring_all_to_floor - Bring all elevators to specified floor')
        self.get_logger().info('  /release_all_elevators - Release all elevators from hold')
    
    def handle_elevator_request(self, request, response):
        """Handle ROS service requests for elevators"""
        # Using AddTwoInts service: request.a = from_floor, request.b = to_floor
        from_floor = request.a
        to_floor = request.b
        
        self.get_logger().info(f'Received elevator request: Floor {from_floor} → Floor {to_floor}')
        
        # Find best elevator
        best_elevator = self.find_best_elevator(from_floor, to_floor)
        
        if best_elevator:
            response.sum = best_elevator  # Return elevator ID
            self.get_logger().info(f'Assigned elevator {best_elevator}')
            
            # Mark as busy
            with self.lock:
                self.elevators[best_elevator].available_for_call = False
        else:
            response.sum = -1  # No elevator available
            self.get_logger().warn('No elevator available for request')
        
        return response
    
    def handle_bring_all_request(self, request, response):
        """Handle request to bring all elevators to a specific floor and hold them there"""
        self.get_logger().info('DEBUG: handle_bring_all_request called!')
        target_floor = request.a
        
        if not (0 <= target_floor <= 3):
            self.get_logger().error(f'Invalid floor: {target_floor}. Must be 0-3.')
            response.sum = 0
            return response
        
        self.get_logger().info(f'Bringing all elevators to floor {target_floor} and holding')
        
        successful_commands = 0
        
        for elevator_id in range(1, self.num_elevators + 1):
            try:
                # Send hold command to each elevator
                client = self.elevator_hold_clients[elevator_id]
                
                if client.wait_for_service(timeout_sec=1.0):
                    hold_request = AddTwoInts.Request()
                    hold_request.a = target_floor
                    
                    future = client.call_async(hold_request)
                    # Note: In production, you'd want to wait for all responses
                    # For now, we'll just count successful sends
                    successful_commands += 1
                    
                    self.get_logger().info(f'Sent hold command to elevator {elevator_id}')
                else:
                    self.get_logger().warn(f'Service not available for elevator {elevator_id}')
            
            except Exception as e:
                self.get_logger().error(f'Failed to send command to elevator {elevator_id}: {e}')
        
        response.sum = successful_commands
        self.get_logger().info(f'Successfully sent commands to {successful_commands}/{self.num_elevators} elevators')
        
        return response
    
    def handle_release_all_request(self, request, response):
        """Handle request to release all elevators from hold"""
        (void_request) = request  # Mark as intentionally unused
        
        self.get_logger().info('Releasing all elevators from hold')
        
        successful_releases = 0
        
        for elevator_id in range(1, self.num_elevators + 1):
            try:
                # Send release command to each elevator
                client = self.elevator_release_clients[elevator_id]
                
                if client.wait_for_service(timeout_sec=1.0):
                    release_request = AddTwoInts.Request()
                    release_request.a = 0  # Dummy value
                    
                    future = client.call_async(release_request)
                    successful_releases += 1
                    
                    self.get_logger().info(f'Released elevator {elevator_id}')
                else:
                    self.get_logger().warn(f'Release service not available for elevator {elevator_id}')
            
            except Exception as e:
                self.get_logger().error(f'Failed to release elevator {elevator_id}: {e}')
        
        response.sum = successful_releases
        self.get_logger().info(f'Successfully released {successful_releases}/{self.num_elevators} elevators')
        
        return response
    
    def send_elevator_to_floor(self, elevator_id: int, target_floor: int) -> bool:
        """Send a specific elevator to a specific floor"""
        if elevator_id not in self.elevator_goto_clients:
            self.get_logger().error(f'No client for elevator {elevator_id}')
            return False
        
        client = self.elevator_goto_clients[elevator_id]
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Goto service not available for elevator {elevator_id}')
            return False
        
        try:
            goto_request = AddTwoInts.Request()
            goto_request.a = target_floor
            
            future = client.call_async(goto_request)
            # In a full implementation, you'd wait for the response
            
            self.get_logger().info(f'Sent goto command to elevator {elevator_id} for floor {target_floor}')
            return True
        
        except Exception as e:
            self.get_logger().error(f'Failed to send goto command to elevator {elevator_id}: {e}')
            return False
    
    def floor_callback(self, msg: Int32, elevator_id: int):
        """Update elevator floor information"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].current_floor = msg.data
                self.elevators[elevator_id].last_update = time.time()
                
                # Infer state from movement
                self.infer_elevator_state(elevator_id)
    
    def door_callback(self, msg: Bool, elevator_id: int):
        """Update elevator door state"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].doors_open = msg.data
                self.elevators[elevator_id].last_update = time.time()
    
    def position_callback(self, msg: PoseStamped, elevator_id: int):
        """Update elevator position"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].position = msg
                self.elevators[elevator_id].last_update = time.time()
    
    def infer_elevator_state(self, elevator_id: int):
        """Infer elevator state from available data"""
        elevator = self.elevators[elevator_id]
        current_time = time.time()
        
        # Simple state inference based on door state and movement
        if elevator.doors_open:
            elevator.state = ElevatorState.IDLE
        else:
            # Doors closed - likely moving or transitioning
            # This is simplified - real implementation would track more state
            elevator.state = ElevatorState.MOVING_UP  # Default assumption
    
    def get_available_elevators(self) -> List[int]:
        """Get list of currently available elevator IDs"""
        self.get_logger().info(f'Getting available elevators')
        # with self.lock:
        available = []
        for elevator_id, elevator in self.elevators.items():
            if elevator.is_available():
                available.append(elevator_id)
        return available
    
    def get_moving_elevators(self) -> List[int]:
        """Get list of currently moving elevator IDs"""
        with self.lock:
            moving = []
            for elevator_id, elevator in self.elevators.items():
                if elevator.is_moving():
                    moving.append(elevator_id)
            return moving
    
    def find_best_elevator(self, requested_floor: int, destination_floor: int) -> Optional[int]:
        """Find the best elevator for a given request"""
        self.get_logger().info(f'Finding best elevator for request: {requested_floor} → {destination_floor}')
        with self.lock:
            available_elevators = self.get_available_elevators()
            self.get_logger().debug(f'Available elevators: {available_elevators}')
            if not available_elevators:
                self.get_logger().warn('No available elevators for request')
                return None
            
            # Simple algorithm: find closest available elevator
            best_elevator = None
            min_distance = float('inf')
            
            for elevator_id in available_elevators:
                elevator = self.elevators[elevator_id]
                distance = abs(elevator.current_floor - requested_floor)
                
                if distance < min_distance:
                    min_distance = distance
                    best_elevator = elevator_id
            
            return best_elevator
    
    def coordinate_elevators(self):
        """Main coordination logic to ensure optimal elevator availability"""
        if not self.coordination_enabled:
            return
        
        with self.lock:
            available_count = len(self.get_available_elevators())
            moving_count = len(self.get_moving_elevators())
            
            self.get_logger().debug(
                f'Coordination check: {available_count} available, {moving_count} moving'
            )
            
            # Ensure minimum number of elevators are available
            if available_count < self.min_available:
                self.get_logger().warn(
                    f'Only {available_count} elevators available, need {self.min_available}'
                )
                # In a real system, you might adjust elevator scheduling here
            
            # Prevent too many elevators moving simultaneously
            if moving_count > self.max_moving:
                self.get_logger().warn(
                    f'{moving_count} elevators moving, max allowed: {self.max_moving}'
                )
                # In a real system, you might delay some elevator movements
    
    def publish_status(self):
        """Publish current system status"""
        with self.lock:
            available_elevators = self.get_available_elevators()
            moving_elevators = self.get_moving_elevators()
            
            # Log summary periodically
            if hasattr(self, '_last_status_log'):
                if time.time() - self._last_status_log > 10.0:  # Every 10 seconds
                    self._log_status_summary(available_elevators, moving_elevators)
                    self._last_status_log = time.time()
            else:
                self._last_status_log = time.time()
    
    def _log_status_summary(self, available: List[int], moving: List[int]):
        """Log a summary of elevator status"""
        status_lines = []
        status_lines.append("=== Elevator System Status ===")
        
        with self.lock:
            for elevator_id, elevator in self.elevators.items():
                doors = "OPEN" if elevator.doors_open else "CLOSED"
                available_status = "AVAILABLE" if elevator.is_available() else "BUSY"
                
                status_lines.append(
                    f"Elevator {elevator_id}: Floor {elevator.current_floor}, "
                    f"Doors {doors}, {available_status}"
                )
        
        status_lines.append(f"Available: {len(available)} | Moving: {len(moving)}")
        status_lines.append("=" * 30)
        
        for line in status_lines:
            self.get_logger().info(line)
    
    def request_elevator(self, from_floor: int, to_floor: int) -> Optional[int]:
        """Request an elevator for robot transportation"""
        best_elevator = self.find_best_elevator(from_floor, to_floor)
        
        if best_elevator:
            self.get_logger().info(
                f'Assigned elevator {best_elevator} for trip: Floor {from_floor} → {to_floor}'
            )
            
            # Mark elevator as busy (in a full implementation, you'd send commands)
            with self.lock:
                self.elevators[best_elevator].available_for_call = False
        else:
            self.get_logger().warn(
                f'No available elevator for trip: Floor {from_floor} → {to_floor}'
            )
        
        return best_elevator
    
    def release_elevator(self, elevator_id: int):
        """Release an elevator back to general availability"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].available_for_call = True
                self.get_logger().info(f'Released elevator {elevator_id} back to service')


def main(args=None):
    """Main function to run the elevator scheduler"""
    rclpy.init(args=args)
    
    try:
        scheduler = ElevatorScheduler()
        
        # Use multi-threaded executor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(scheduler)
        
        scheduler.get_logger().info('Elevator Scheduler started - monitoring system...')
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            scheduler.get_logger().info('Elevator Scheduler shutting down...')
        finally:
            executor.shutdown()
            scheduler.destroy_node()
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()