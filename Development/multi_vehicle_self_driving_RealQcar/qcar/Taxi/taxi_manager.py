"""
Pure Python Taxi Manager (Replaces trip_planner.py ROS 2 node)
Orchestrates multi-stop missions/trips for the QCar.

Quanser Competition Scenario Flow:
  1. Start at Taxi Hub (Node 10) — LED Magenta (idle)
  2. Drive to pick-up node          — LED Green  (driving)
  3. Full stop at pick-up           — LED Blue   (passenger pick-up, wait 3s)
  4. Drive to drop-off node         — LED Green  (driving)
  5. Full stop at drop-off          — LED Orange (passenger drop-off, wait 3s)
  6. Drive back to Taxi Hub         — LED Green  (driving)
  7. Arrive at Hub                  — LED Magenta (ready for next ride)

For trips with more than 2 user-supplied nodes the intermediate stops are
treated as additional pick-ups / drop-offs following the same pattern.

Stop types exposed via ``get_current_stop_type()``:
  "hub"     — at the taxi hub (idle / ready)
  "pickup"  — first user stop (passenger boarding)
  "dropoff" — last user stop  (passenger alighting)
  "intermediate" — any stop between pickup and dropoff
  "driving" — currently in motion
"""

import time
from typing import List, Optional, ClassVar
from enum import IntEnum

try:
    from Taxi.led_client import LedClient
except ImportError:
    try:
        from led_client import LedClient
    except ImportError:
        LedClient = None  # LED control unavailable


class TripSuperState(IntEnum):
    """Super state mapping for the overarching status of the taxi"""

    INITIALIZING = 1  # Going to taxi hub / Initializing
    SERVING_RIDES = 2  # Ready for rides / Serving rides


class TripState(IntEnum):
    """State mapping inside a specific ride/trip"""

    IDLE = 0  # idle at hub
    DRIVING = 1  # driving
    WAITING = 2  # waiting at stop (pickup/dropoff)


class TaxiManager:
    """
    Manages taxi trips logic.
    Maintains a queue of nodes and handles the wait times at each stop.
    """

    # Default scenario: Node 20 = pick-up, Node 9 = drop-off, Node 10 = hub
    DEFAULT_TRIP_NODES: List[int] = [20, 9]

    def __init__(
        self,
        taxi_hub_node: int = 10,
        stop_wait_time: float = 3.0,
        default_trip_nodes: Optional[List[int]] = None,
    ):
        self.taxi_hub_node = taxi_hub_node
        self.stop_wait_time = stop_wait_time  # seconds to wait at each stop
        self.default_trip_nodes = (
            list(default_trip_nodes)
            if default_trip_nodes
            else list(self.DEFAULT_TRIP_NODES)
        )

        self.trip_super_state = TripSuperState.INITIALIZING

        self.current_trip_state = TripState.IDLE

        self.trip_nodes: List[int] = []  # user-requested nodes (without hub)
        self.path_nodes: List[int] = []  # full path: hub + trip_nodes + hub

        self.current_trip_status = False  # True if a trip is active
        self.stop_index = 0  # index into path_nodes of *current* origin

        self.wait_timer_start = 0.0
        self.waiting_at_stop = False

        # Stop-type tracking (for LED / telemetry)
        self._current_stop_type: str = "hub"

        # LED control via UDP bridge to ROS 2 qcar2_hardware node
        self._led_client: Optional["LedClient"] = None
        if LedClient is not None:
            try:
                self._led_client = LedClient()
                print("TaxiManager: LED client connected (UDP → taxi_led_bridge).")
            except Exception as e:
                print(f"TaxiManager: LED client init failed: {e}")
        self._update_led()  # set initial LED (magenta = hub/idle)

    # ------------------------------------------------------------------
    # Trip lifecycle
    # ------------------------------------------------------------------

    def request_new_trip(self, nodes: List[int]) -> bool:
        """
        Request a new trip.  Must be at the taxi hub (trip_super_state == TripSuperState.SERVING_RIDES).
        ``nodes`` are the user-requested intermediate stops (e.g. [2, 5]).
        The manager automatically prepends/appends the hub node and removes
        consecutive duplicates so users can include node 10 without issues.

        Returns True if the trip was accepted.
        """
        print(nodes, "nodes")
        if self.trip_super_state == TripSuperState.INITIALIZING:
            print(
                "TaxiManager: Cannot assign trip, not at the taxi hub! "
                "Please route to hub first."
            )
            return False

        if self.current_trip_status:
            print("TaxiManager: Cannot assign trip, current trip in progress!")
            return False

        if len(nodes) < 1:
            print("TaxiManager: Invalid trip — at least 1 destination node required.")
            return False

        self.trip_nodes = list(nodes)

        # Build full path: Hub → [user nodes] → Hub, then deduplicate consecutive
        raw_path = [self.taxi_hub_node] + self.trip_nodes + [self.taxi_hub_node]
        self.path_nodes = [raw_path[0]]
        for node in raw_path[1:]:
            if node != self.path_nodes[-1]:
                self.path_nodes.append(node)

        print(f"TaxiManager: New trip requested! Path: {self.path_nodes}")
        self.current_trip_status = True
        self.stop_index = 0
        self.current_trip_state = TripState.DRIVING  # driving to first stop
        self._current_stop_type = "driving"
        self._update_led()

        return True

    def request_default_trip(self) -> bool:
        """Start the default scenario trip (pickup node 20 → dropoff node 9 → hub)."""
        print(
            f"TaxiManager: Starting default trip with nodes {self.default_trip_nodes}"
        )
        return self.request_new_trip(self.default_trip_nodes)

    def dispatch_to_hub(self, current_node: int = None):
        """Dispatches the taxi to the hub if it's currently initializing (state 1)."""
        if self.trip_super_state == TripSuperState.INITIALIZING:
            if self.current_trip_status:
                print("TaxiManager: Already handling a trip to the hub!")
                return False

            print(f"TaxiManager: Dispatching to Taxi Hub (Node {self.taxi_hub_node})")
            self.trip_nodes = []

            # If current_node is provided and not the hub, route from there to the hub
            if current_node is not None and current_node != self.taxi_hub_node:
                self.path_nodes = [current_node, self.taxi_hub_node]
            else:
                # Fallback: just use [hub] and let the state machine handle arrival immediately
                self.path_nodes = [self.taxi_hub_node]

            self.current_trip_status = True
            self.stop_index = 0
            self.current_trip_state = TripState.DRIVING
            self._current_stop_type = "driving"
            self._update_led()
            return True
        else:
            print("TaxiManager: Already at the hub or serving rides.")
            return False

    def cancel_current_trip(self):
        """Cancels any ongoing trip."""
        self.current_trip_status = False
        self.trip_nodes = []
        self.path_nodes = []
        self.current_trip_state = TripState.IDLE
        self.waiting_at_stop = False
        self.trip_super_state = (
            TripSuperState.INITIALIZING
        )  # Reset — require routing to hub again
        self._current_stop_type = "hub"
        self._update_led()

    # ------------------------------------------------------------------
    # Arrival & waiting
    # ------------------------------------------------------------------

    def handle_stop_arrival(self) -> bool:
        """
        Called when the vehicle reaches the current destination node.
        Returns True if the trip is fully completed, False if more stops remain.
        """
        # Arriving at hub during initialisation phase
        if self.trip_super_state == TripSuperState.INITIALIZING:
            self.set_arrived_at_hub()  # also updates LED to hub/magenta
            self.current_trip_status = False
            self.trip_nodes = []
            self.path_nodes = []
            return True

        # Advance to the next segment
        self.stop_index += 1

        if self.stop_index >= len(self.path_nodes) - 1:
            # Reached the final node (taxi hub)
            print("TaxiManager: Trip complete. Back at Taxi Hub.")
            self.current_trip_status = False
            self.trip_nodes = []
            self._current_stop_type = "hub"
            self.current_trip_state = TripState.IDLE
            self._update_led()
            return True

        # Determine stop type for the node we just arrived at
        self._current_stop_type = self._classify_stop(self.stop_index)
        print(
            f"TaxiManager: Arrived at stop {self.stop_index} "
            f"(node {self.path_nodes[self.stop_index]}, "
            f"type={self._current_stop_type}). Waiting {self.stop_wait_time}s…"
        )

        self.waiting_at_stop = True
        self.wait_timer_start = time.time()
        self.current_trip_state = TripState.WAITING  # waiting
        self._update_led()

        return False

    def check_wait_time(self) -> bool:
        """
        Checks if the wait time at a stop has elapsed.
        Returns True if wait is over (or not waiting), False otherwise.
        """
        if not self.waiting_at_stop:
            return True

        if time.time() - self.wait_timer_start >= self.stop_wait_time:
            self.waiting_at_stop = False
            self.current_trip_state = TripState.DRIVING  # back to driving
            self._current_stop_type = "driving"
            self._update_led()
            return True

        return False

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_next_destination(self) -> int:
        """Returns the next destination node, or -1 if no active trip."""
        if not self.current_trip_status or self.stop_index >= len(self.path_nodes) - 1:
            return -1
        return self.path_nodes[self.stop_index + 1]

    def get_current_segment(self) -> List[int]:
        """Returns ``[start_node, end_node]`` for the current leg."""
        if not self.current_trip_status or self.stop_index >= len(self.path_nodes) - 1:
            return []

        start = self.path_nodes[self.stop_index]
        end = self.path_nodes[self.stop_index + 1]
        return [start, end]

    def get_current_stop_type(self) -> str:
        """
        Returns the semantic type of the current stop for LED / telemetry:
          "hub", "pickup", "dropoff", "intermediate", or "driving"
        """
        return self._current_stop_type

    def get_trip_progress(self) -> dict:
        """Return a summary dict useful for telemetry / GUI."""
        total_segments = max(len(self.path_nodes) - 1, 0)
        return {
            "active": self.current_trip_status,
            "super_state": self.trip_super_state.value,
            "trip_state": self.current_trip_state.value,
            "stop_type": self._current_stop_type,
            "segment": self.stop_index,
            "total_segments": total_segments,
            "path_nodes": list(self.path_nodes),
            "waiting": self.waiting_at_stop,
        }

    # ------------------------------------------------------------------
    # Hub helpers
    # ------------------------------------------------------------------

    def check_hub_arrival(
        self,
        curr_x: float,
        curr_y: float,
        hub_x: float,
        hub_y: float,
        threshold: float = 0.5,
    ) -> bool:
        """Check if vehicle is physically at the hub."""
        dist = ((curr_x - hub_x) ** 2 + (curr_y - hub_y) ** 2) ** 0.5
        if dist < threshold and self.trip_super_state == TripSuperState.INITIALIZING:
            self.set_arrived_at_hub()
            return True
        return False

    def set_arrived_at_hub(self):
        """Marks the vehicle as having arrived at the hub initially."""
        if self.trip_super_state == TripSuperState.INITIALIZING:
            self.trip_super_state = TripSuperState.SERVING_RIDES
            self.current_trip_state = TripState.IDLE
            self._current_stop_type = "hub"
            self._update_led()
            print("TaxiManager: Arrived at Taxi Hub. Ready for rides.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _classify_stop(self, stop_idx: int) -> str:
        """
        Classify a stop by its position in the path.

        path_nodes layout:  [hub, pickup, ..., dropoff, hub]
        indices:              0     1    ...    N-2      N-1

        - index 0         → hub (start)
        - index 1         → pickup  (first user node)
        - index N-2       → dropoff (last user node)
        - index N-1       → hub (return)
        - anything else   → intermediate
        """
        n = len(self.path_nodes)
        if stop_idx <= 0 or stop_idx >= n - 1:
            return "hub"
        if stop_idx == 1:
            return "pickup"
        if stop_idx == n - 2:
            return "dropoff"
        return "intermediate"

    # ------------------------------------------------------------------
    # LED control
    # ------------------------------------------------------------------

    def _update_led(self):
        """
        Push the current stop-type to the hardware LED strip via the
        UDP bridge (taxi_led_bridge ROS 2 node).

        Mapping (see led_client.py STOP_TYPE_TO_LED):
            hub          → 5 (magenta)
            driving      → 1 (green)
            pickup       → 2 (blue)
            dropoff      → 3 (yellow / orange)
            intermediate → 4 (cyan)
        """
        if self._led_client is not None:
            try:
                self._led_client.set_color(self._current_stop_type)
            except Exception as e:
                print(f"TaxiManager: LED update failed: {e}")
