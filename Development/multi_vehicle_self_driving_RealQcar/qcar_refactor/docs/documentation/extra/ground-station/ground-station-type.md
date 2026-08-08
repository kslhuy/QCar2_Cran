# `extra/ground_station/ground_station_type.py`

## Purpose

Owns every operator-side ground-station data contract:
`GroundStationConfiguration`, `GroundStationCommandRequest`, `VehicleSession`,
`DisconnectedVehicle`, and `CommandDelivery`. Core listener/session services
and terminal presentation consume these types but do not define them.

## Verification

`test/unit_test_ground_station.py` asserts the defining module for every
ground-station dataclass.
