import re
import matplotlib.pyplot as plt
from datetime import datetime

# Lists to store data
sent_times, sent_x, sent_y, sent_z, sent_v = [], [], [], [], []
recv_times, recv_x, recv_y, recv_z = [], [], [], []

# Regular expressions to match SENT and RECEIVED STATE lines
sent_pattern = re.compile(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}).*SENT: Seq: \d+, Pos: \[([-.\d]+), ([-.\d]+), ([-.\d]+)\], V: ([-.\d]+)')
recv_pattern = re.compile(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}).*RECEIVED STATE: Seq: \d+, Sender ID: \d+, Pos: \[([-.\d]+), ([-.\d]+), ([-.\d]+)\]')

# Read the log file
with open('Development/fleet_framwork/logs/communication_vehicle_0.log', 'r') as file:
    for line in file:
        # Match SENT lines
        sent_match = sent_pattern.match(line)
        if sent_match:
            time_str, x, y, z, v = sent_match.groups()
            time = datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S,%f')
            sent_times.append(time)
            sent_x.append(float(x))
            sent_y.append(float(y))
            sent_z.append(float(z))
            sent_v.append(float(v))
        
        # Match RECEIVED STATE lines
        recv_match = recv_pattern.match(line)
        if recv_match:
            time_str, x, y, z = recv_match.groups()
            time = datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S,%f')
            recv_times.append(time)
            recv_x.append(float(x))
            recv_y.append(float(y))
            recv_z.append(float(z))

# Convert times to seconds relative to the first timestamp
sent_times_sec = [(t - sent_times[0]).total_seconds() for t in sent_times]
recv_times_sec = [(t - sent_times[0]).total_seconds() for t in recv_times]

# Create plots
plt.figure(figsize=(10, 6))

# Plot X position vs Time
plt.subplot(1, 3, 1)
plt.plot(sent_times_sec, sent_x, label='Sent X', marker='o', color='blue')
plt.plot(recv_times_sec, recv_x, label='Received X', marker='x', color='red')
plt.xlabel('Time (seconds)')
plt.ylabel('X Position')
plt.title('X Position vs Time')
plt.legend()
plt.grid(True)

# Plot Y position vs Time
plt.subplot(1, 3, 2)
plt.plot(sent_times_sec, sent_y, label='Sent Y', marker='o', color='blue')
plt.plot(recv_times_sec, recv_y, label='Received Y', marker='x', color='red')
plt.xlabel('Time (seconds)')
plt.ylabel('Y Position')
plt.title('Y Position vs Time')
plt.legend()
plt.grid(True)

# Plot Velocity vs Time (only for sent data)
plt.subplot(1, 3, 3)
plt.plot(sent_times_sec, sent_v, label='Sent Velocity', marker='o', color='green')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity')
plt.title('Velocity vs Time')
plt.legend()
plt.grid(True)

plt.tight_layout()

# Create 2D XY trajectory plot
plt.figure(figsize=(6, 6))
plt.scatter(sent_x, sent_y, label='Vehicle 0 (Sent)', color='blue', marker='o', s=50)
plt.scatter(recv_x, recv_y, label='Vehicle 1 (Received)', color='red', marker='x', s=50)
plt.plot(sent_x, sent_y, color='blue', alpha=0.5, linestyle='-', linewidth=1)
plt.plot(recv_x, recv_y, color='red', alpha=0.5, linestyle='-', linewidth=1)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('XY Trajectory')
plt.legend()
plt.grid(True)

plt.show()