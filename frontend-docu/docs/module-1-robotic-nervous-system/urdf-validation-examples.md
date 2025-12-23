# URDF Validation Examples

This document provides examples and best practices for validating URDF files for humanoid robots.

## Basic URDF Validation Commands

### 1. Syntax Validation
```bash
# Check basic URDF syntax
check_urdf simple_humanoid.urdf

# Expected output should show:
# robot name is: simple_humanoid
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 6 child links
```

### 2. Kinematic Tree Generation
```bash
# Generate kinematic tree visualization
urdf_to_graphiz simple_humanoid.urdf

# This creates:
# - simple_humanoid.gv (Graphviz file)
# - simple_humanoid.pdf (Visual diagram)
```

## Common URDF Issues and Solutions

### Issue 1: Missing Joint Definitions
**Problem:**
```xml
<!-- Missing parent/child links -->
<joint name="missing_links" type="revolute">
    <!-- No parent or child specified -->
</joint>
```

**Solution:**
```xml
<joint name="correct_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.0 -0.05 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
</joint>
```

### Issue 2: Inconsistent Mass Properties
**Problem:**
```xml
<link name="leg_link">
    <inertial>
        <mass value="0"/>  <!-- Mass cannot be zero -->
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>  <!-- All zeros -->
    </inertial>
</link>
```

**Solution:**
```xml
<link name="leg_link">
    <inertial>
        <mass value="3.0"/>  <!-- Positive mass value -->
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>  <!-- Calculated values -->
    </inertial>
</link>
```

### Issue 3: Invalid Joint Limits
**Problem:**
```xml
<joint name="bad_limits" type="revolute">
    <parent link="base_link"/>
    <child link="hip"/>
    <limit lower="1.0" upper="0.5" effort="100" velocity="1"/>  <!-- Lower > Upper -->
</joint>
```

**Solution:**
```xml
<joint name="good_limits" type="revolute">
    <parent link="base_link"/>
    <child link="hip"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>  <!-- Proper range -->
</joint>
```

## Validation Script Example

Here's a comprehensive validation script:

```python
#!/usr/bin/env python3
import subprocess
import xml.etree.ElementTree as ET
import sys
import os

def validate_humanoid_urdf(urdf_path):
    """Comprehensive validation for humanoid URDF files"""
    print(f"Validating humanoid URDF: {urdf_path}")

    # 1. Basic syntax validation
    print("\n1. Checking URDF syntax...")
    try:
        result = subprocess.run(['check_urdf', urdf_path],
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ Syntax validation passed")
        else:
            print(f"✗ Syntax validation failed: {result.stderr}")
            return False
    except FileNotFoundError:
        print("✗ check_urdf command not found - is ROS installed?")
        return False

    # 2. Parse XML structure
    print("\n2. Analyzing URDF structure...")
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Check for robot name
        robot_name = root.get('name')
        if not robot_name:
            print("✗ Missing robot name")
            return False
        print(f"✓ Robot name: {robot_name}")

        # Find all links and joints
        links = root.findall('.//link')
        joints = root.findall('.//joint')

        print(f"✓ Found {len(links)} links and {len(joints)} joints")

        # Check for essential humanoid components
        link_names = [link.get('name').lower() for link in links]
        joint_names = [joint.get('name').lower() for joint in joints]

        essential_checks = [
            ('torso', any('torso' in name or 'base' in name for name in link_names)),
            ('head', any('head' in name for name in link_names)),
            ('left leg', any('left' in name and ('hip' in name or 'leg' in name) for name in joint_names)),
            ('right leg', any('right' in name and ('hip' in name or 'leg' in name) for name in joint_names)),
            ('left foot', any('left' in name and 'foot' in name for name in link_names)),
            ('right foot', any('right' in name and 'foot' in name for name in link_names)),
        ]

        print("\n3. Essential humanoid components:")
        all_present = True
        for component, present in essential_checks:
            status = "✓" if present else "✗"
            print(f"  {status} {component}: {'Present' if present else 'Missing'}")
            if not present:
                all_present = False

        if not all_present:
            print("✗ Some essential humanoid components are missing")
            return False

        # Check joint limits validity
        print("\n4. Validating joint limits...")
        joints_with_issues = []

        for joint in joints:
            joint_type = joint.get('type')
            if joint_type in ['revolute', 'prismatic']:  # These have limits
                limit_elem = joint.find('limit')
                if limit_elem is not None:
                    lower = float(limit_elem.get('lower', 0))
                    upper = float(limit_elem.get('upper', 0))

                    if lower > upper:
                        joints_with_issues.append(f"{joint.get('name')}: lower({lower}) > upper({upper})")

        if joints_with_issues:
            print("✗ Joint limit issues found:")
            for issue in joints_with_issues:
                print(f"    - {issue}")
            return False
        else:
            print("✓ All joint limits are valid")

        # Check mass properties
        print("\n5. Validating mass properties...")
        links_with_issues = []

        for link in links:
            inertial = link.find('inertial')
            if inertial is not None:
                mass_elem = inertial.find('mass')
                if mass_elem is not None:
                    mass = float(mass_elem.get('value', 0))
                    if mass <= 0:
                        links_with_issues.append(f"{link.get('name')}: mass must be positive")

        if links_with_issues:
            print("✗ Mass property issues found:")
            for issue in links_with_issues:
                print(f"    - {issue}")
            return False
        else:
            print("✓ All mass properties are valid")

        print("\n✓ All validations passed! URDF is ready for humanoid robot applications.")
        return True

    except ET.ParseError as e:
        print(f"✗ XML parsing error: {e}")
        return False
    except Exception as e:
        print(f"✗ Validation error: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 validate_humanoid_urdf.py <urdf_file>")
        sys.exit(1)

    urdf_file = sys.argv[1]
    if not os.path.exists(urdf_file):
        print(f"Error: File {urdf_file} does not exist")
        sys.exit(1)

    success = validate_humanoid_urdf(urdf_file)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
```

## Running the Validation

Save the validation script as `validate_humanoid_urdf.py` and run:

```bash
chmod +x validate_humanoid_urdf.py
python3 validate_humanoid_urdf.py your_humanoid_robot.urdf
```

## Common Validation Scenarios

### Scenario 1: Bipedal Walking Robot
For a bipedal humanoid robot, ensure:
- Hip joints have appropriate range for walking
- Knee joints only bend in one direction (positive angles)
- Ankle joints have limited range for stability
- Mass distribution supports bipedal balance

### Scenario 2: Upper Body Mobility
For humanoid robots with arm mobility:
- Shoulder joints should have 3DOF (or multiple 1DOF joints)
- Elbow joints should have appropriate flexion range
- Hand/wrist joints should support manipulation tasks

### Scenario 3: Simulation vs Real Robot
Remember to validate for both contexts:
- **Simulation**: Can use simplified geometries
- **Real Robot**: Must match physical constraints and safety limits

## Troubleshooting Common Errors

### Error: "No matching function" in Gazebo
- Solution: Ensure all joint types are supported by your simulator
- Check that plugin configurations are correct

### Error: "Joint state not found"
- Solution: Verify joint names match between URDF and controller configuration
- Check that joint state publisher is properly configured

### Error: "Inertia matrix not positive definite"
- Solution: Ensure inertia values follow the rules for physical objects
- Use proper formulas for basic geometric shapes

This validation process ensures your humanoid robot URDF is correct and ready for both simulation and real-world applications.