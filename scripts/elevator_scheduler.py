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
from dataclasses import dataclass, field
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

class SingleElevatorRequestState(Enum):
    IDLE = "IDLE"
    EN_ROUTE = "EN_ROUTE"
    TRANSPORTING = "TRANSPORTING"

class SingleElevatorRequestDirection(Enum):
    UP = 1
    DOWN = -1

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

@dataclass
class SingleElevatorRequest:
    """Tracks a single elevator request"""
    from_floor: int = None
    direction: SingleElevatorRequestDirection = None
    to_floors: list[int] = field(default_factory=list)  # List of target floors
    elevator: Optional[ElevatorStatus] = None

    def clear(self):
        self.from_floor = None
        self.direction = None
        self.to_floors = []
        self.elevator = None
    
    def add_stop(self, floor: int):
        if self.direction == SingleElevatorRequestDirection.UP and \
           floor < self.elevator.current_floor:
            raise Exception('Invalid stop request')

        if self.direction == SingleElevatorRequestDirection.DOWN and \
           floor > self.elevator.current_floor:
            raise Exception('Invalid stop request')

        if floor not in self.to_floors:
            self.to_floors.append(floor)
        
            if self.direction == SingleElevatorRequestDirection.UP:
                self.to_floors.sort()
            elif self.direction == SingleElevatorRequestDirection.DOWN:
                self.to_floors.sort(reverse=True)   

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

        # ROS Service for adding a stop
        self.add_stop_service = self.create_service(
            AddTwoInts,
            'add_stop',
            self.handle_add_stop_request
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
        
        # TODO: Fix publishers for elevator status
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

        self._single_elevator_request_state = SingleElevatorRequestState.IDLE
        self.single_elevator_request = SingleElevatorRequest()  # Track which elevator is handling the request

        self.get_logger().info(f'Elevator Scheduler initialized for {self.num_elevators} elevators')
        self.get_logger().info(f'Coordination: {self.coordination_enabled}, Min available: {self.min_available}')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /request_elevator - Request elevator for robot transport')
        self.get_logger().info('  /bring_all_to_floor - Bring all elevators to specified floor')
        self.get_logger().info('  /release_all_elevators - Release all elevators from hold')
    
    def handle_elevator_request(self, request, response):
        """Handle ROS service requests for elevators"""
        # Using AddTwoInts service: request.a = from_floor
        self.get_logger().info(f'Received elevator request: Floor {request.a}, Direction {request.b}')

        from_floor = request.a

        if request.b not in [SingleElevatorRequestDirection.UP.value, SingleElevatorRequestDirection.DOWN.value]:
            self.get_logger().error(f'Invalid direction: {request.b}. Must be 1 (up) or -1 (down).')
            response.sum = -1
            return response

        response.sum = -1  # Default response

        # TODO: Validate floor requests

        # check if single elevator request is available
        if self.single_elevator_request_state == SingleElevatorRequestState.IDLE:
            # Find best elevator
            best_elevator = self.find_best_elevator(from_floor)
            
            if best_elevator:
                response.sum = best_elevator  # Return elevator ID
                self.get_logger().info(f'Assigned elevator {best_elevator}')
                
                # Mark as busy
                with self.lock:
                    self.elevators[best_elevator].available_for_call = False

                # Set single elevator request
                with self.lock:
                    self.single_elevator_request.from_floor = from_floor
                    self.single_elevator_request.direction = SingleElevatorRequestDirection(request.b)
                    self.single_elevator_request.elevator = self.elevators[best_elevator]

                # Set single elevator request state to EN_ROUTE
                def set_en_route():
                    self.single_elevator_request_state = SingleElevatorRequestState.EN_ROUTE
                threading.Thread(target=set_en_route, daemon=True).start()
            else:
                self.get_logger().warn('No elevator available for request')
        else:
            self.get_logger().warn('Elevator is busy with another request')
        
        return response

    def handle_add_stop_request(self, request, response):
        """Handle request to add a stop to the current elevator"""
        self.get_logger().info('Received add stop request')
        target_floor = request.a

        if self.single_elevator_request_state != SingleElevatorRequestState.TRANSPORTING:
            self.get_logger().error('Invalid state for adding stop: must be TRANSPORTING')
            response.sum = -1
            return response

        try:
            self.single_elevator_request.add_stop(target_floor)
            self.get_logger().info(f'Added floor {target_floor} to stop list for elevator {self.single_elevator_request.elevator.elevator_id}')
            response.sum = 1
        except Exception as e:
            self.get_logger().error(f'Failed to add stop: {e}')
            response.sum = -1

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
    
    def hold_elevator_at_floor(self, elevator_id: int, target_floor: int) -> bool:
        """Send a specific elevator to a specific floor"""
        if elevator_id not in self.elevator_goto_clients:
            self.get_logger().error(f'No client for elevator {elevator_id}')
            return False
        
        # Use hold request to send elevator for pickup
        client = self.elevator_hold_clients[elevator_id]
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Hold service not available for elevator {elevator_id}')
            return False
        
        try:
            hold_request = AddTwoInts.Request()
            hold_request.a = target_floor

            future = client.call_async(hold_request)
            # In a full implementation, you'd wait for the response

            self.get_logger().info(f'Sent hold command to elevator {elevator_id} for floor {target_floor}')
            return True
        
        except Exception as e:
            self.get_logger().error(f'Failed to send hold command to elevator {elevator_id}: {e}')
            return False
    
    def floor_callback(self, msg: Int32, elevator_id: int):
        """Update elevator floor information"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].current_floor = msg.data
                self.elevators[elevator_id].last_update = time.time()
                
                # Infer state from movement
                self.infer_elevator_single_elevator_request_state(elevator_id)
    
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
    
    def infer_elevator_single_elevator_request_state(self, elevator_id: int):
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
    
    def find_best_elevator(self, requested_floor: int) -> Optional[int]:
        """Find the best elevator for a given request"""
        with self.lock:
            available_elevators = self.get_available_elevators()
            self.get_logger().debug(f'Available elevators: {available_elevators}')
            if not available_elevators:
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
    
    @property
    def single_elevator_request_state(self):
        return self._single_elevator_request_state

    @single_elevator_request_state.setter
    def single_elevator_request_state(self, new_state):
        if self._single_elevator_request_state != new_state:
            self._single_elevator_request_state = new_state
            self.on_update_for_single_elevator_request()
    
    def on_update_for_single_elevator_request(self):
        if self.single_elevator_request_state == SingleElevatorRequestState.IDLE:
            self.get_logger().info('Elevator is idle, ready for requests')
            self.handle_idle_single_elevator_request_state()
        elif self.single_elevator_request_state == SingleElevatorRequestState.EN_ROUTE:
            self.get_logger().info('Elevator is en route to requested floor')
            self.handle_en_route_single_elevator_request_state()
        elif self.single_elevator_request_state == SingleElevatorRequestState.TRANSPORTING:
            self.get_logger().info('Elevator is ready to transport passengers')
            self.handle_transporting_single_elevator_request_state()
    
    def handle_idle_single_elevator_request_state(self):
        # Send release command to each elevator
        elevator_id = self.single_elevator_request.elevator.elevator_id
        client = self.elevator_release_clients[elevator_id]

        if client.wait_for_service(timeout_sec=1.0):
            release_request = AddTwoInts.Request()
            release_request.a = 0  # Dummy value
            
            future = client.call_async(release_request)
            
            self.get_logger().info(f'Released elevator {elevator_id}')
        self.elevators[self.single_elevator_request.elevator.elevator_id].available_for_call = True
        self.single_elevator_request.clear()
        self.get_logger().info('Single elevator request cleared and ready for new requests')
    
    def handle_en_route_single_elevator_request_state(self):
        self.get_logger().info(f'Elevator {self.single_elevator_request.elevator.elevator_id} is en route!')

        # Send elevator to the requested floor
        self.hold_elevator_at_floor(
            self.single_elevator_request.elevator.elevator_id,
            self.single_elevator_request.from_floor
        )

        # Wait for elevator to arrive at the requested floor
        timeout = 60  # seconds
        start_time = time.time()
        elevator = self.single_elevator_request.elevator
        # TODO: Use a more robust way to check if the elevator has arrived
        while not (elevator.current_floor == self.single_elevator_request.from_floor and elevator.doors_open):
            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    f"Timeout waiting for elevator {elevator.elevator_id} to reach floor "
                    f"{self.single_elevator_request.from_floor}"
                )
                self.single_elevator_request_state = SingleElevatorRequestState.IDLE
                return
            time.sleep(0.1)
        self.get_logger().info(
            f'Elevator {elevator.elevator_id} has reached floor {self.single_elevator_request.from_floor}'
        )

        # Set single elevator request state to TRANSPORTING
        self.single_elevator_request_state = SingleElevatorRequestState.TRANSPORTING

    def handle_transporting_single_elevator_request_state(self):
        self.get_logger().info('Waiting for target floors to be added...')

        # Wait for target floors to be added
        wait_timeout = 30  # seconds
        start_time = time.time()
        elevator = self.single_elevator_request.elevator
        while not self.single_elevator_request.to_floors:
            if time.time() - start_time > wait_timeout:
                self.get_logger().warn('Timeout waiting for target floors to be added')
                self.single_elevator_request_state = SingleElevatorRequestState.IDLE
                return
            time.sleep(0.1)

        # Start transporting passengers
        self.get_logger().info('Starting to transport passengers...')
        while self.single_elevator_request.to_floors:
            next_floor = self.single_elevator_request.to_floors.pop(0)
            self.get_logger().info(f'Sending elevator to floor {next_floor}')
            self.send_elevator_to_floor(
                self.single_elevator_request.elevator.elevator_id,
                next_floor
            )
            while not (elevator.current_floor == next_floor and elevator.doors_open):
                time.sleep(0.1)
            self.get_logger().info(
                f'Elevator {elevator.elevator_id} has reached floor {next_floor}'
            )
        self.get_logger().info('All requested stops completed.')
        
        # After all stops are done, reset state
        self.single_elevator_request_state = SingleElevatorRequestState.IDLE


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