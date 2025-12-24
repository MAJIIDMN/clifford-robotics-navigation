# Robot Geometric Algebra (robot_ga)

Robot simulation menggunakan Geometric Algebra untuk simulasi gerakan 2D. Project ini merupakan implementasi dari konsep Geometric Algebra dalam ROS 2 untuk simulasi pergerakan robot dengan transformasi rotasi yang tepat.

| Nama | NIM |
|------|-----|
| Muhammad Nur Majiid | 13524028 |

---

## Daftar Isi

1. [Project Description](#project-description)
2. [Dependencies](#dependencies)
3. [Project Structure](#project-structure)
4. [Installation](#installation)
5. [Working Directory](#working-directory)
6. [How to Run](#how-to-run)
7. [Nodes](#nodes)
8. [Topics](#topics)
9. [Troubleshooting](#troubleshooting)

---

## Project Description

Project ini mengimplementasikan simulasi robot dengan menggunakan konsep **Geometric Algebra** (GA) untuk menangani transformasi rotasi dan translasi 2D. Sistem ini menggunakan **ROS 2** sebagai middleware komunikasi antar node.

### Fitur Utama:
- Simulasi gerakan robot 2D dengan geometric algebra
- Implementasi Rotor 2D untuk rotasi yang akurat
- Publikasi path dan pose real-time
- Visualisasi menggunakan RViz2
- Integrasi dengan ROS 2 ecosystem

---

## Dependencies

### System Requirements
- **ROS 2**: Humble atau versi lebih baru
- **Python**: 3.10 atau lebih baru
- **NumPy**: untuk perhitungan matematika
- **Ubuntu**: 20.04 LTS atau lebih baru (recommended: 22.04 LTS)

### ROS 2 Dependencies
Semua dependencies sudah terdaftar dalam `package.xml`:

```xml
<exec_depend>rclpy</exec_depend>           <!-- Python ROS 2 Client Library -->
<exec_depend>geometry_msgs</exec_depend>   <!-- Twist, PoseStamped messages -->
<exec_depend>nav_msgs</exec_depend>        <!-- Path message untuk trajectory -->
<exec_depend>tf2_ros</exec_depend>         <!-- Transform Broadcasting -->
<exec_depend>rviz2</exec_depend>           <!-- RViz visualization tool -->
```

### Python Dependencies
- `numpy`: Perhitungan numerik dan linear algebra
- `setuptools`: Build system untuk package

### Development Dependencies (Testing)
```xml
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
<test_depend>ament_pep257</test_depend>
<test_depend>python3-pytest</test_depend>
```

### Cara Install Dependencies

**1. Install ROS 2 Humble (jika belum ada):**
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rviz2
```

**2. Install Python dependencies:**
```bash
pip install numpy
# atau jika menggunakan conda:
conda install numpy
```

**3. Install ROS 2 Python development tools:**
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

---

## Project Structure

```
robot_ga/
├── README.md                           # Project documentation
├── LICENSE                             # Apache-2.0 License
├── package.xml                         # ROS 2 package metadata
├── setup.py                            # Python package setup
├── setup.cfg                           # Setup configuration
├── robot_ga/                           # Main package directory
│   ├── __init__.py                    # Package initialization
│   ├── ga_math.py                     # Geometric Algebra math library
│   │                                   # - Implementasi Rotor2D
│   │                                   # - Fungsi rotasi 2D
│   └── motion_node.py                 # Main motion node
│                                       # - Subscripe ke /cmd_vel
│                                       # - Publikasi /pose dan /path
│                                       # - Transform broadcasting
├── launch/                            # Launch files
│   └── ga_robot_launch.py            # Main launch configuration
│                                       # - Memulai motion_node
│                                       # - Memulai RViz2
├── rviz/                              # RViz configuration
│   └── config.rviz                   # RViz display configuration
├── resource/                          # Package resources
│   └── robot_ga                      # Package resource marker
└── test/                              # Test files
    ├── test_copyright.py             # Copyright test
    ├── test_flake8.py                # Code style test
    └── test_pep257.py                # Documentation style test
```

---

## Installation

### Step 1: Clone Repository
```bash
git clone https://github.com/MAJIIDMN/robot_ga.git
cd robot_ga
```

### Step 2: Setup ROS 2 Environment
Pastikan Anda sudah men-source ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Install Package Dependencies
```bash
rosdep install --from-paths . --ignore-src -r -y
```

### Step 4: Build Package
```bash
colcon build --symlink-install
```

### Step 5: Source Workspace Setup
```bash
source install/setup.bash
```

---

## Working Directory

### Workspace Structure
```
ros_workspace/
├── src/
│   ├── robot_ga/                      # Source folder project ini
│   └── ...                            # Package ROS lain
├── build/                             # Build artifacts (auto-generated)
├── install/                           # Installed packages (auto-generated)
└── log/                               # Build logs (auto-generated)
```

### Recommended Working Directory Setup
```bash
# 1. Buat workspace baru (jika belum ada)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 2. Clone project ke src folder
cd src
git clone https://github.com/MAJIIDMN/robot_ga.git

# 3. Kembali ke workspace root
cd ..

# 4. Build
colcon build

# 5. Source setup
source install/setup.bash
```

### Environment Variables
Setelah source setup.bash, environment variables berikut akan ter-set:
```bash
ROS_PACKAGE_PATH      # Path ke package ROS
PYTHONPATH            # Path ke Python modules
CMAKE_PREFIX_PATH     # Path ke CMake modules
```

Untuk check environment, jalankan:
```bash
echo $ROS_PACKAGE_PATH
printenv | grep ROS
```

---

## How to Run

### Prerequisites
Pastikan:
1. ROS 2 Humble sudah terinstall
2. Python 3.10+ terinstall
3. NumPy sudah installed (`pip install numpy`)
4. Package sudah di-build (`colcon build`)

### Method 1: Using Launch File (Recommended)

```bash
# Terminal 1: Setup environment
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 1: Launch everything (motion_node + rviz2)
ros2 launch robot_ga ga_robot_launch.py
```

Ini akan automatically memulai:
- GAMotionNode (subscribed ke /cmd_vel)
- RViz2 (visualization tool)
- Transform broadcaster

### Method 2: Run Individual Nodes

**Terminal 1: Run motion node**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_ga motion_node
```

**Terminal 2: Run RViz2 (optional)**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 -d install/robot_ga/share/robot_ga/rviz/config.rviz
```

**Terminal 3: Send command velocity (untuk testing)**
```bash
# Publish Twist message untuk menggerakkan robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

## Nodes

### GAMotionNode

**Executable**: `motion_node`

**Description**: Node utama yang menangani simulasi gerakan robot 2D menggunakan Geometric Algebra.

**Subscriptions**:
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Command velocity (linear.x, angular.z) |

**Publications**:
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/pose` | `geometry_msgs/PoseStamped` | Current robot pose (x, y, orientation) |
| `/path` | `nav_msgs/Path` | Robot trajectory history |
| `/tf` | `tf2_msgs/TFMessage` | Transform broadcasts |

**Parameters**: None (fixed update rate 50Hz)

**Implementation Details**:
- Update frequency: 50 Hz (dt = 0.05s)
- Menggunakan Rotor2D untuk rotasi akurat
- Maintains pose, heading, dan velocity
- Path history untuk trajectory visualization

**Sample Usage**:
```bash
# Check published topics
ros2 topic list

# Echo pose in real-time
ros2 topic echo /pose

# Check node info
ros2 node info /ga_motion_node
```

---

## Topics

### Published Topics

#### `/pose` - Robot Pose
```
Type: geometry_msgs/PoseStamped
Frame: map
Rate: 20 Hz (update setiap 0.05s)

Structure:
  header:
    frame_id: "map"
    stamp: <timestamp>
  pose:
    position: {x: float, y: float, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: sin(theta/2), w: cos(theta/2)}
```

#### `/path` - Robot Path/Trajectory
```
Type: nav_msgs/Path
Frame: map
Rate: 20 Hz

Structure:
  header:
    frame_id: "map"
  poses: [array of PoseStamped messages]
```

#### `/tf` - Transforms
```
Type: tf2_msgs/TFMessage
Broadcasts transform dari 'map' ke 'base_link'
```

### Subscribed Topics

#### `/cmd_vel` - Command Velocity
```
Type: geometry_msgs/Twist

Format:
  linear:
    x: forward/backward velocity (m/s)
    y: (tidak digunakan, always 0)
    z: (tidak digunakan, always 0)
  angular:
    x: (tidak digunakan, always 0)
    y: (tidak digunakan, always 0)
    z: angular velocity (rad/s)
```

---

## Mathematical Implementation

### Geometric Algebra Concepts

**Rotor 2D**: Digunakan untuk merepresentasikan rotasi dalam 2D
```python
class Rotor2D:
    def __init__(self, theta):
        # theta: rotation angle in radians
        self.c = cos(theta/2)  # cosine component
        self.s = sin(theta/2)  # sine component
    
    def rotate(self, v):
        # v: [x, y] vector
        # Returns: rotated vector
```

**Motion Model**:
```
theta(t+1) = theta(t) + omega * dt
direction = Rotor2D(theta).rotate([1, 0])  # unit vector in robot frame
pose(t+1) = pose(t) + v * direction * dt
```

---

## Troubleshooting

### Issue 1: "Package 'robot_ga' not found"
**Cause**: Workspace tidak di-source atau package tidak di-build
**Solution**:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_ga ga_robot_launch.py
```

### Issue 2: "ModuleNotFoundError: No module named 'numpy'"
**Cause**: NumPy tidak terinstall
**Solution**:
```bash
pip install numpy
# atau
sudo apt install python3-numpy
```

### Issue 3: RViz tidak bisa menampilkan robot
**Cause**: RViz config file tidak ditemukan atau TF tidak di-broadcast
**Solution**:
```bash
# Verify TF adalah published:
ros2 topic echo /tf

# Verify geometry_msgs package installed:
ros2 pkg list | grep geometry_msgs
```

### Issue 4: Node subscriptions tidak menerima message
**Cause**: `/cmd_vel` topic tidak ada atau tidak publishing
**Solution**:
```bash
# Check active topics
ros2 topic list

# Publish test message
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

### Issue 5: Build error "ament_cmake not found"
**Cause**: Development tools ROS 2 tidak installed
**Solution**:
```bash
sudo apt install python3-colcon-common-extensions
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Running Tests

```bash
# Run all tests
colcon test

# Run specific test file
colcon test --packages-select robot_ga --ctest-args -R test_flake8

# View test results
colcon test-result
```

---

## Code Quality

Project mengikuti Python style guide:
- **Flake8**: Code style checking
- **PEP257**: Documentation style checking
- **Copyright**: License header checking

Check code quality:
```bash
flake8 src/robot_ga/
```

---

## References

- [ROS 2 Documentation](https://docs.ros.org/)
- [Geometric Algebra Introduction](https://en.wikipedia.org/wiki/Geometric_algebra)
- [ROS 2 Python Client Library](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html)
- [RViz User Guide](https://wiki.ros.org/rviz)

---

## License

Project ini dilisensikan di bawah Apache License 2.0. Lihat file `LICENSE` untuk detail lebih lanjut.

---

## Author

**Muhammad Nur Majiid**
- NIM: 13524028
- Email: muhammadnurmajiid22@gmail.com
- GitHub: [@MAJIIDMN](https://github.com/MAJIIDMN)

---

## Questions & Support

Untuk pertanyaan atau masalah, silakan:
1. Buka GitHub Issues di repository
2. Hubungi penulis via email

---

**Last Updated**: December 2025  
**ROS 2 Version**: Humble (Ubuntu 22.04)
**Python Version**: 3.10+
