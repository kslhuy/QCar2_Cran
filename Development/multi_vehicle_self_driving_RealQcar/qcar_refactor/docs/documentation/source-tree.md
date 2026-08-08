# Source Tree to Documentation Map

```text
core/
  commands.py                 -> core/commands.md
  command_handler.py          -> core/command-handler.md
  module_factory.py           -> core/module-factory.md
  vehicle_config.py           -> core/vehicle-config.md
  vehicle_logic.py            -> core/vehicle-runtime.md
  vehicle_main.py             -> core/vehicle-main.md
  vehicle_process.py          -> core/vehicle-process.md
  vehicle_state_machine.py    -> core/vehicle-state-machine.md
  vehicle_types.py            -> core/vehicle-types.md
utils/
  control/                    -> utils/control/README.md
  fleet/                      -> utils/fleet/README.md
  ground_station/             -> utils/ground-station/README.md
  io/                         -> utils/io/README.md
  v2v/                        -> utils/v2v/README.md
extra/
                              -> extra/README.md
  ground_station/             -> extra/ground-station/README.md
  simulator/                  -> extra/simulator/README.md
config/
                              -> config/README.md (current profiles/scenarios only)
```

Each package index maps every substantive source file to its page. Import-only
`__init__.py` files are intentionally not documented individually. The two
empty ROS IO adapters share one stub page, and the tiny distributed-observer
factory is documented with its interface. `config/scenarios/test/` is excluded
from this project-documentation map and belongs with its automated tests.
