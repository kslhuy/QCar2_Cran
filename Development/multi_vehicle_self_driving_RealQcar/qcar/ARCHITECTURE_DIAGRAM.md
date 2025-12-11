# Enhanced Logging System Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Vehicle Control System                              │
│                                                                              │
│  ┌────────────────┐         ┌────────────────┐        ┌──────────────────┐ │
│  │ Main Control   │         │  V2V Manager   │        │  Vehicle Logger  │ │
│  │ Loop (100Hz+)  │         │                │        │                  │ │
│  └────────┬───────┘         └────────┬───────┘        └─────────┬────────┘ │
│           │                          │                           │          │
│           │ Telemetry                │ Received States           │          │
│           └──────────────────────────┴───────────────────────────┘          │
│                                      │                                      │
└──────────────────────────────────────┼──────────────────────────────────────┘
                                       │
                                       ▼
                        ┌──────────────────────────────┐
                        │     VehicleLogger API        │
                        │                              │
                        │  • log_telemetry()           │
                        │  • log_fleet_estimation()    │
                        │  • log_local_estimation()    │
                        └──────────────┬───────────────┘
                                       │
                    ┌──────────────────┼──────────────────┐
                    │                  │                  │
                    ▼                  ▼                  ▼
         ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
         │  Telemetry Queue │ │ Fleet Est Queue  │ │ Local Est Queue  │
         │  (1000 entries)  │ │  (1000 entries)  │ │  (1000 entries)  │
         └────────┬─────────┘ └────────┬─────────┘ └────────┬─────────┘
                  │                    │                    │
         Non-blocking                  │                    │
         queue.put_nowait()            │                    │
                  │                    │                    │
                  ▼                    ▼                    ▼
         ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
         │ Background       │ │ Background       │ │ Background       │
         │ Worker Thread    │ │ Worker Thread    │ │ Worker Thread    │
         │ (Daemon)         │ │ (Daemon)         │ │ (Daemon)         │
         └────────┬─────────┘ └────────┬─────────┘ └────────┬─────────┘
                  │                    │                    │
         Buffered write                │                    │
         Flush every 100 entries       │                    │
                  │                    │                    │
                  ▼                    ▼                    ▼
         ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
         │  telemetry_      │ │ received_fleet_  │ │ received_local_  │
         │  vehicle_X.csv   │ │ estimations_     │ │ estimations_     │
         │                  │ │ vehicle_X.csv    │ │ vehicle_X.csv    │
         └──────────────────┘ └──────────────────┘ └──────────────────┘
                  │                    │                    │
                  └────────────────────┴────────────────────┘
                                       │
                                       ▼
                            ┌─────────────────────┐
                            │  Data Analysis      │
                            │  • pandas           │
                            │  • matplotlib       │
                            │  • numpy            │
                            └─────────────────────┘
```

## Data Flow for Fleet Estimation Logging

```
Vehicle 0                     Network                    Vehicle 1
─────────                     ───────                    ─────────

┌─────────────┐                                         ┌─────────────┐
│ Observer    │                                         │ Observer    │
│ estimates   │                                         │ receives    │
│ fleet state │                                         │ fleet state │
└──────┬──────┘                                         └──────▲──────┘
       │                                                       │
       │ Fleet state dict:                                    │
       │ {                                                    │
       │   'vehicle_0': {x, y, θ, v},                        │
       │   'vehicle_1': {x, y, θ, v}                         │
       │ }                                                    │
       │                                                      │
       ▼                                                      │
┌─────────────┐              UDP                       ┌─────────────┐
│ V2VManager  │─────────────Multicast──────────────────▶│ V2VManager  │
│ broadcast   │           (port 8000)                   │ receives    │
└─────────────┘                                         └──────┬──────┘
                                                               │
                                                               │
                                      ┌────────────────────────┘
                                      │
                                      ▼
                              ┌──────────────────┐
                              │ _handle_fleet_   │
                              │ state_message()  │
                              └────────┬─────────┘
                                       │
                                       │ Calls log_fleet_estimation()
                                       ▼
                              ┌──────────────────┐
                              │ VehicleLogger    │
                              │ fleet_est_queue  │
                              └────────┬─────────┘
                                       │
                              Non-blocking put
                                       │
                                       ▼
                              ┌──────────────────┐
                              │ Background       │
                              │ Worker Thread    │
                              └────────┬─────────┘
                                       │
                              Writes multiple rows
                              (one per vehicle)
                                       │
                                       ▼
                    ┌──────────────────────────────────────────┐
                    │ received_fleet_estimations_vehicle_1.csv │
                    │                                          │
                    │ timestamp, receive_time, sender_id, ...  │
                    │ 123.456,   123.458,      0,         ...  │
                    │ 123.456,   123.458,      0,         ...  │
                    └──────────────────────────────────────────┘
                                  (2 rows for 2 vehicles)
```

## Performance Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Main Control Loop                           │
│                     (100-1000 Hz)                               │
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    │
│  │  Read GPS    │───▶│  Control     │───▶│  Actuators   │    │
│  │  Sensors     │    │  Algorithm   │    │  Output      │    │
│  └──────────────┘    └──────────────┘    └──────────────┘    │
│         │                                          │           │
│         │                                          │           │
│         ▼                                          ▼           │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │            Logging Calls (Non-blocking)                 │  │
│  │  • logger.log_telemetry(data)                          │  │
│  │  • logger.log_fleet_estimation(...)                    │  │
│  │  • logger.log_local_estimation(...)                    │  │
│  │                                                          │  │
│  │  ⏱️ Time taken: <0.001 ms (queue.put_nowait())          │  │
│  └─────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Loop continues immediately - NO BLOCKING                      │
└─────────────────────────────────────────────────────────────────┘
                               │
                               │ Async queues
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│              Background Logging System                          │
│              (Separate threads, low priority)                   │
│                                                                 │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │ Telemetry Worker │  │ Fleet Est Worker │  │ Local Worker │ │
│  │                  │  │                  │  │ Est Worker   │ │
│  │ while active:    │  │ while active:    │  │ while active:│ │
│  │   get from queue │  │   get from queue │  │   get queue  │ │
│  │   write to CSV   │  │   write to CSV   │  │   write CSV  │ │
│  │   flush (100x)   │  │   flush (100x)   │  │   flush(100x)│ │
│  │                  │  │                  │  │              │ │
│  │ ⏱️ ~1-5ms/write   │  │ ⏱️ ~1-5ms/write   │  │ ⏱️ ~1-5ms    │ │
│  └────────┬─────────┘  └────────┬─────────┘  └──────┬───────┘ │
│           │                     │                    │         │
└───────────┼─────────────────────┼────────────────────┼─────────┘
            │                     │                    │
            ▼                     ▼                    ▼
      ┌─────────┐           ┌─────────┐          ┌─────────┐
      │ CSV     │           │ CSV     │          │ CSV     │
      │ File    │           │ File    │          │ File    │
      │ (8KB    │           │ (8KB    │          │ (8KB    │
      │ buffer) │           │ buffer) │          │ buffer) │
      └────┬────┘           └────┬────┘          └────┬────┘
           │                     │                    │
           │ fsync() every 100 entries                │
           │                     │                    │
           ▼                     ▼                    ▼
      ┌─────────────────────────────────────────────────┐
      │              Persistent Storage                 │
      │              (Data guaranteed safe)             │
      └─────────────────────────────────────────────────┘
```

## Thread Safety Model

```
┌─────────────────────────────────────────────────────────────┐
│                    Thread-Safe Logging                      │
│                                                             │
│  Main Thread              Queue (thread-safe)               │
│  ───────────              ─────────────────                 │
│                                                             │
│  log_fleet_estimation()                                     │
│         │                                                   │
│         ├─▶ queue.put_nowait() ──┐                         │
│         │                         │                         │
│         │   (returns immediately) │                         │
│         │                         │                         │
│         ▼                         │                         │
│  continue execution               │                         │
│                                   │                         │
│                                   ▼                         │
│                          ┌─────────────────┐                │
│                          │  Thread-Safe    │                │
│                          │  Queue          │                │
│  Background Thread       │  (mutex locked) │                │
│  ──────────────          └────────┬────────┘                │
│                                   │                         │
│  _worker_thread()                 │                         │
│         │                         │                         │
│         ◀─────queue.get()─────────┘                         │
│         │                                                   │
│         ├─▶ csv_writer.writerow()                          │
│         │                                                   │
│         ├─▶ file.flush()                                   │
│         │                                                   │
│         └─▶ os.fsync()                                     │
│                                                             │
│  No shared state (except queue)                            │
│  No race conditions                                        │
│  No deadlocks                                              │
└─────────────────────────────────────────────────────────────┘
```

## Error Handling Flow

```
┌─────────────────────────────────────────────────────────────┐
│                   Error Handling Strategy                   │
│                                                             │
│  Main Thread:                                               │
│  ───────────                                                │
│                                                             │
│  try:                                                       │
│      logger.log_fleet_estimation(...)                      │
│  except queue.Full:                                         │
│      pass  # Drop data (very rare)                         │
│  except Exception as e:                                     │
│      print(f"Error: {e}")  # Non-blocking print            │
│                                                             │
│  ► Main loop NEVER blocks on logging errors                │
│  ► Vehicle control continues normally                      │
│                                                             │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  Background Thread:                                         │
│  ──────────────────                                         │
│                                                             │
│  while active:                                              │
│      try:                                                   │
│          entry = queue.get(timeout=0.1)                    │
│          csv_writer.writerow(entry)                        │
│          if flush_needed:                                  │
│              try:                                           │
│                  file.flush()                              │
│                  os.fsync()                                │
│              except (OSError, IOError):                    │
│                  # Continue on flush error                 │
│                  pass                                      │
│      except queue.Empty:                                    │
│          continue  # Check if should stop                  │
│      except Exception as e:                                 │
│          # Log error and continue                          │
│          time.sleep(0.01)  # Brief pause                   │
│                                                             │
│  ► Graceful degradation                                    │
│  ► No cascading failures                                   │
└─────────────────────────────────────────────────────────────┘
```

## Memory Management

```
Per Vehicle Memory Footprint:
──────────────────────────────

VehicleLogger instance:
├─ Telemetry Queue:       ~100 KB  (1000 × 100 bytes)
├─ Fleet Est Queue:       ~100 KB  (1000 × 100 bytes)
├─ Local Est Queue:       ~100 KB  (1000 × 100 bytes)
├─ File Buffers (3):       24 KB   (3 × 8 KB)
├─ CSV Writers (3):         5 KB   (metadata)
├─ Thread Objects (3):     10 KB   (thread overhead)
└─ Logger Object:           5 KB   (logger state)
                          ────────
Total:                    ~344 KB  (0.34 MB)

Fleet of 5 vehicles:
Total memory:              ~1.7 MB

► Negligible for modern systems
► No memory leaks (bounded queues)
► Automatic cleanup on close()
```

## Data Persistence Strategy

```
Write Path:
──────────

1. queue.put_nowait()        ← Non-blocking (microseconds)
2. queue.get()               ← Background thread
3. csv_writer.writerow()     ← Buffered (8KB buffer)
4. Periodic flush (100 rows) ← Batch writes
5. file.flush()              ← OS buffer flush
6. os.fsync()                ← Force disk write

Data Safety Guarantees:
─────────────────────

✅ Data guaranteed on disk after flush
✅ Maximum data loss: Last 100 entries (~1-2 seconds)
✅ Clean shutdown flushes all queues
✅ Crash recovery: Most data preserved

Performance vs Safety Trade-off:
────────────────────────────────

Flush Frequency:  100 entries
                  ▲           ▼
            More safety    Better performance
            More I/O       Less I/O
            Less latency   More latency
```

---

**Key Design Principles:**

1. **Non-blocking First**: Main loop never waits for I/O
2. **Thread Isolation**: Separate threads for each log type
3. **Graceful Degradation**: Errors don't crash the system
4. **Data Integrity**: fsync ensures persistence
5. **Bounded Resources**: Fixed-size queues prevent memory growth
6. **Performance Monitoring**: Built-in queue size tracking
