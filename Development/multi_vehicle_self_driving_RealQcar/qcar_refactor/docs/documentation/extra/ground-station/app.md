# `extra/ground_station/app.py`

## Purpose

Provides the canonical operator command:

```text
python -m extra.ground_station terminal
python -m extra.ground_station server
```

It dispatches to the terminal or headless implementation. Both load the same
fixed-path `config/ground_station.yaml` file without a selector argument.
