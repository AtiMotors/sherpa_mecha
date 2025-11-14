# Sherpa Robot Models

This repository provides ROS 2 description packages for the **Asystr Sensible** robot and its dual-arm configuration attached to an AMR **Sherpa**.  
It is intended for visualization, simulation, and further extension with motion planning and control frameworks (e.g., MoveIt).  

---

## Packages

### 1. `asystr_sensible_model`
- Contains the **URDF model** and launch files for a single **Asystr Sensible** arm.  
- Supports different robot setups:
  - **`base`** – Minimal model without wrist (default).
  - **`wrist` / `full`** – Model with wrist attached.
- Wrist configuration modes:
  - **`RPR`** (default)
  - **`YPR`**
- Includes a **`chiral` parameter** to mirror the robot about the Z-axis of `base_link`.
  - This is useful for creating left/right arm variants.

### 2. `asystr_dual_arm_model`
- Constructs a **dual-arm mobile robot** using `asystr_sensible_model`.  
- Attaches left and right arms to the **Sherpa** mobile base (AMR)
- Wrist configuration can be selected per arm:
  - **`RPR`** (default)  
  - **`YPR`**  

---

## URDF Files

### Pre-generated URDFs
- The **Sherpa AMR URDF** is located at:  
  `asystr_dual_arm_model/urdf/sherpa.urdf`   
  (generated from `sherpa_dual_arm.urdf.xacro`)

---

### Generate URDF from Xacro

If you need to regenerate a `.urdf` file from its `.xacro` source:  

#### 1. Install the `xacro` package
```bash
sudo apt install ros-${ROS_DISTRO}-xacro
```

#### 2. Source your ROS installation
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

#### 3. Source your workspace
(from the root of your workspace, after `colcon build`)
```bash
source install/setup.bash
```

#### 4. Run the xacro conversion
**ROS 2 command:**
```bash
ros2 run xacro xacro <path_to>/<model>.urdf.xacro -o <path_to>/<model>.urdf
```

**Alternative (direct):**
```bash
xacro <path_to>/<model>.urdf.xacro -o <path_to>/<model>.urdf
```

**Example:**
```bash
ros2 run xacro xacro src/asystr_dual_arm_model/urdf/sherpa_dual_arm.urdf.xacro -o src/asystr_dual_arm_model/urdf/sherpa.urdf
```

---

## Launch Instructions

### Dual Arm (Sherpa)
```bash
ros2 launch asystr_dual_arm_model sherpa_rviz.launch.py
```
