## Consensus Term
- [] Tranfer the estimated state from the distributed observer to be fleet state. **The meaning of them are different**

    The estimated state from the distributed observer:
    - pi - pi-1 + di0
    - vi - v0
    - ai - a0

    The fleet state:
    ```
        Expected data structure:
        {
            'sender_id': int,
            'fleet_states': {
                vehicle_id: {
                    'x': float, 
                    'y': float, 
                    'theta': float, 
                    'velocity': float,
                    'confidence': float
                },
                ...
            },
            'source': 'fleet_consensus'
        }
    ```
    Method: def _transfer_estimated_states_to_fleet_states(self, estimated_state: np.ndarray) -> np.ndarray:

    **Question**: If we calcultate pi = pi-1-di0, where the pi-1 should come from? from the local data or fleet data? I think it should be local data. di0 also.
            