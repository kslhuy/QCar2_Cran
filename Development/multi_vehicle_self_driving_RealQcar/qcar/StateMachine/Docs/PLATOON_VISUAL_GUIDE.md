# Platoon Command Flow - Visual Guide

## State Machine: Platoon Flow with Setup Tracking

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│              🚗 VEHICLE STATE MACHINE - PLATOON CONTROL             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

INITIAL STATE: WAITING_FOR_START (or FOLLOWING_PATH)
     ↓
     │  setup_complete: FALSE ❌
     │  formation_data: {}
     │  my_position: None
     ↓

┌──────────────────────────────────────────────────────────────────────────┐
│                    COMMAND: SETUP_PLATOON_FORMATION                     │
│  {                                                                        │
│    "type": "setup_platoon_formation",                                    │
│    "formation": {                                                         │
│      0: 1,    ← Vehicle 0 is LEADER (position 1)                        │
│      1: 2,    ← Vehicle 1 is FOLLOWER (position 2)                      │
│      2: 3     ← Vehicle 2 is FOLLOWER (position 3)                      │
│    },                                                                     │
│    "leader_id": 0                                                         │
│  }                                                                        │
└──────────────────────────────────────────────────────────────────────────┘
     ↓
     │ [Process in state_base.py]
     │
     │ platoon_controller.setup_from_global_formation(
     │     my_car_id=1,
     │     formation={0: 1, 1: 2, 2: 3},
     │     leader_id=0
     │ )
     ↓
┌──────────────────────────────────────────────────────────────────────────┐
│                    FOR VEHICLE 1 (FOLLOWER):                            │
│                                                                           │
│  self.is_leader = False                                                   │
│  self.leader_car_id = 0                                                   │
│  self.my_position = 2                ← VEHICLE 1's POSITION             │
│                                                                           │
│  🔑 CRITICAL: setup_complete = TRUE ✅                                   │
│                                                                           │
│  self.formation_data = {0: 1, 1: 2, 2: 3}                                 │
│  self.expected_followers = []                                             │
│                                                                           │
│  LOG: ✅ Configured from global formation: Role=FOLLOWER (position 2),  │
│       Leader=0, setup_complete=True                                      │
└──────────────────────────────────────────────────────────────────────────┘
     ↓
     │  NOW READY TO START PLATOON! setup_complete = TRUE ✅
     ↓

     🟢 NEXT COMMAND: START_PLATOON (when ready)

     ┌──────────────────────────────────────────────────────────────────┐
     │                    COMMAND: START_PLATOON                        │
     │  {                                                                 │
     │    "type": "start_platoon",                                       │
     │    "leader_id": 0                                                 │
     │  }                                                                 │
     └──────────────────────────────────────────────────────────────────┘
          ↓
          │ [Check in following_path_state.py or waiting_for_start_state.py]
          │
          │ if not platoon_controller.setup_complete:
          │     LOG: ⚠️ START_PLATOON rejected - setup not complete!
          │     RETURN None  ← IGNORE THIS COMMAND
          │ else:
          │     PROCEED WITH TRANSITION
          ↓
          ✅ CHECK PASSED: setup_complete = TRUE
          ↓
          if is_leader:
              → FOLLOW_PATH (leader waits for followers)
          else:
              → FOLLOWING_LEADER (follower starts following)
          ↓
┌──────────────────────────────────────────────────────────────────────────┐
│                    NEW STATE: FOLLOWING_LEADER                           │
│                                                                           │
│  Follower is now:                                                         │
│  ✅ Connected to leader (Vehicle 0)                                      │
│  ✅ Monitoring distance via perception/V2V                              │
│  ✅ Adjusting velocity to maintain spacing                              │
│  ✅ Ready for platoon control                                           │
└──────────────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════

## FAILURE CASE: Command Sequence Error (BEFORE FIX)

WITHOUT setup_complete flag ❌:

WAITING_FOR_START
     ↓
     │  ❌ BAD: Send START_PLATOON directly (without SETUP first!)
     ↓
     │  Vehicle doesn't know:
     │  - Is it a leader or follower?
     │  - Who is my leader?
     │  - What's my position in platoon?
     │  
     │  State transitions ANYWAY (BUG!)
     ↓
🔴 UNDEFINED BEHAVIOR: Wrong state, wrong role, broken platoon


═══════════════════════════════════════════════════════════════════════════════

## SUCCESS CASE: With Setup Validation (AFTER FIX) ✅

WITH setup_complete flag ✅:

WAITING_FOR_START
     ↓
     │  ❌ BAD: Send START_PLATOON directly (without SETUP first!)
     ↓
     │ Check: if setup_complete?
     │    NO! setup_complete = FALSE
     ↓
     │ LOG: ⚠️ START_PLATOON rejected - SETUP_PLATOON_FORMATION not received!
     │
     │ → RETURN None  ← IGNORE COMMAND
     ↓
✅ CORRECT BEHAVIOR: Stay in WAITING_FOR_START, await proper setup command

Then, when SETUP_PLATOON_FORMATION is sent:
     ↓
     │ setup_complete = TRUE ✅
     ↓
     │ Now START_PLATOON is accepted!
     ↓
✅ STATE TRANSITION: WAITING_FOR_START → FOLLOWING_LEADER


═══════════════════════════════════════════════════════════════════════════════

## Command Validation Matrix

┌─────────────────────────────┬──────────────┬─────────┬──────────────────────┐
│ Command                     │ State        │ setup_  │ Result               │
│                             │              │ complete│                      │
├─────────────────────────────┼──────────────┼─────────┼──────────────────────┤
│ SETUP_PLATOON_FORMATION     │ Any          │ False   │ ✅ PROCESS           │
│                             │              │         │ Set setup_complete   │
│                             │              │         │ = TRUE               │
├─────────────────────────────┼──────────────┼─────────┼──────────────────────┤
│ START_PLATOON               │ WAITING      │ False   │ ❌ REJECT            │
│                             │ or PATH      │         │ Log: Setup needed    │
│                             │              │         │ Return None          │
├─────────────────────────────┼──────────────┼─────────┼──────────────────────┤
│ START_PLATOON               │ WAITING      │ True    │ ✅ PROCESS           │
│                             │ or PATH      │         │ Transition to        │
│                             │              │         │ FOLLOWING_LEADER or  │
│                             │              │         │ FOLLOWING_PATH       │
├─────────────────────────────┼──────────────┼─────────┼──────────────────────┤
│ DISABLE_PLATOON             │ Any          │ Any     │ ✅ PROCESS           │
│                             │              │         │ Set setup_complete   │
│                             │              │         │ = FALSE              │
└─────────────────────────────┴──────────────┴─────────┴──────────────────────┘


═══════════════════════════════════════════════════════════════════════════════

## Timeline: Correct Platoon Formation

TIME  │ VEHICLE 0 (LEADER)           │ VEHICLE 1 (FOLLOWER)
──────┼──────────────────────────────┼────────────────────────────
T0    │ WAITING_FOR_START            │ WAITING_FOR_START
      │ setup_complete: False         │ setup_complete: False
──────┼──────────────────────────────┼────────────────────────────
T1    │ RCV: SETUP_PLATOON_FORMATION │ RCV: SETUP_PLATOON_FORMATION
      │ (pos=1, is_leader=True)      │ (pos=2, is_leader=False)
      │ setup_complete: True ✅       │ setup_complete: True ✅
──────┼──────────────────────────────┼────────────────────────────
T2    │ RCV: START_PLATOON           │ RCV: START_PLATOON
      │ Check: setup_complete = True │ Check: setup_complete = True
      │ ✅ ACCEPT                    │ ✅ ACCEPT
──────┼──────────────────────────────┼────────────────────────────
T3    │ → FOLLOWING_PATH (leader)    │ → FOLLOWING_LEADER
      │                              │   (following leader 0)
──────┼──────────────────────────────┼────────────────────────────
T4    │ Moving at formation_speed    │ Detecting leader via YoLO
      │ Waiting for followers ready  │ Adjusting velocity to match
──────┼──────────────────────────────┼────────────────────────────
T5+   │ 🚗 Platoon ACTIVE            │ 🚗 Platoon ACTIVE
      │ (controlling followers)      │ (following leader)
```

