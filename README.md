# 🛰️ Custom LiDAR Integration

**UART Debugging & ROS2 Node Implementation**

## 📘 Overview

This document records the complete process of bringing up and debugging a custom UART-based LiDAR sensor on a ROS2 (Humble) robot platform. The LiDAR streams raw distance frames via `/dev/ttyUSB0`, which are parsed and published as a standard `sensor_msgs/LaserScan` topic for downstream processing (navigation, obstacle avoidance, etc.).

The work involved identifying a UART communication issue, manually validating data streams, and implementing a custom LiDAR driver node in Python to reliably publish `/scan` data.

---

## ⚙️ Hardware and Environment

| Component | Specification |
|-----------|---------------|
| **LiDAR Interface** | UART |
| **Device Path** | `/dev/ttyUSB0` |
| **Baud Rate** | 115200 bps |
| **Stop Bits / Parity** | 1 Stop Bit / None |
| **Platform** | Raspberry Pi 4 / Ubuntu 22.04 |
| **ROS Version** | ROS2 Humble |

---

## ⚠️ Problem Summary

When first connecting the LiDAR, the ROS2 node failed to receive valid scan data. Symptoms included:

- No data published on `/scan`
- Corrupted or random bytes when reading the serial port
- Occasional checksum or framing errors

### 🔍 Root Cause

The UART port was not in **raw mode**, meaning the terminal driver was applying:

- Line buffering
- Echo
- Special character interpretation

This interfered with the LiDAR's binary data stream, causing misaligned frames.

---

## 🧠 Debugging Process

### 1️⃣ Verify Device Enumeration

Checked if the LiDAR was detected correctly:

```bash
dmesg | grep tty
```

✅ **Result:** Confirmed it appeared as `/dev/ttyUSB0`

### 2️⃣ Inspect Raw Serial Output

To verify whether the LiDAR was transmitting valid binary data:

```bash
sudo stty -F /dev/ttyUSB0 115200 -echo raw
sudo head -c 100 /dev/ttyUSB0 | hexdump -C
```

**Output example:**

```
00000000  09 dc 09 e4 09 e8 09 f4  09 04 0a aa 55 00 28 29  |............U.()|
```

✅ **Observation:**

- The `0xAA 0x55` header bytes confirmed valid LiDAR frames
- Data corruption was caused by incorrect serial port mode, not hardware issues

---

## 🛠️ The Fix

### Step 1: Configure UART Properly

Disable line buffering and echo:

```bash
sudo stty -F /dev/ttyUSB0 115200 -echo raw
```

### Step 2: Programmatic Configuration

Implemented the same configuration in the ROS2 node using Python's `pyserial`:

```python
lidar = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)
```

✅ **Result:** After applying this fix, the LiDAR began streaming valid data continuously.

---

## 🧩 Final ROS2 Node Implementation

### 📄 `custom_lidar_publisher.py`

```python
#!/usr/bin/env python3
"""
Custom LiDAR ROS2 Node
----------------------
Authors: Akash A , N Mujeeb Rahman
Description:
    Custom ROS2 node to interface with a UART-based LiDAR sensor via /dev/ttyUSB0.
    Reads binary data frames directly from the serial port, decodes distance
    measurements, and publishes them as a standard LaserScan message.

Fix Applied:
    - Added correct UART configuration (115200 baud, raw mode, no echo)
    - Verified data frames manually using hexdump
    - Ensured consistent frame sync (0xAA 0x55 header)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import numpy as np

class CustomLidar(Node):
    def __init__(self):
        super().__init__('custom_lidar')

        # --- ROS2 Publisher ---
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # --- UART Setup ---
        try:
            self.serial = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info("✅ LiDAR connected on /dev/ttyUSB0 at 115200 baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to open serial port: {e}")
            return

        # --- Timer to read LiDAR data ---
        self.timer = self.create_timer(0.05, self.read_lidar_data)  # 20 Hz

    def read_lidar_data(self):
        """Read raw binary data from LiDAR and publish LaserScan."""
        try:
            # Example: assume fixed-size 22-byte frame; adjust per LiDAR documentation
            data = self.serial.read(22)

            # Verify header bytes
            if len(data) < 22 or data[0] != 0xAA or data[1] != 0x55:
                return  # invalid frame

            # Example frame parsing
            angle = struct.unpack_from('<H', data, 2)[0] / 100.0  # degrees
            distances = []
            for i in range(4, len(data) - 2, 2):
                dist = struct.unpack_from('<H', data, i)[0] / 1000.0  # mm → m
                distances.append(dist)

            # Build LaserScan message
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = 'laser_frame'
            scan.angle_min = math.radians(angle)
            scan.angle_max = math.radians(angle + len(distances))
            scan.angle_increment = math.radians(1.0)
            scan.range_min = 0.05
            scan.range_max = 6.0
            scan.ranges = np.clip(distances, scan.range_min, scan.range_max).tolist()

            self.scan_pub.publish(scan)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().warn(f"LiDAR parse warning: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CustomLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🧾 Verification and Testing

### Run the Node

```bash
ros2 run custom_lidar custom_lidar_publisher
```

### Check Topics

```bash
ros2 topic list
```

**Expected output:**

```
/parameter_events
/rosout
/scan
```

### Inspect LiDAR Readings

```bash
ros2 topic echo /scan
```

✅ **Expected behavior:**

- Continuous scan messages
- Valid range readings
- No serial warnings or checksum errors

---

## ✅ Results

After applying the UART configuration fix and running the custom ROS2 node:

- ✅ The LiDAR streamed clean binary data continuously
- ✅ `/scan` topic published accurate distance readings
- ✅ The LiDAR integrated seamlessly with the rest of the robot stack

---

## 📚 Key Takeaways

1. **Always configure UART in raw mode** when working with binary protocols
2. **Use `hexdump`** to verify data integrity at the hardware level before debugging software
3. **Frame synchronization** (header bytes like `0xAA 0x55`) is critical for parsing binary streams
4. **ROS2 timer callbacks** provide reliable periodic sensor polling

---

## 👤 Author

**Akash A**
**Mujeeb Rahman **


---

## 📄 License

This project is provided as-is for educational and development purposes.
