# Validation Scripts for Module 1

This document provides validation scripts to verify the examples and code provided in Module 1.

## 1. Code Example Validation Script

```bash
#!/bin/bash
# validate_examples.sh - Script to validate ROS 2 code examples

echo "Validating ROS 2 Humble examples for Module 1..."

# Check if ROS 2 Humble is sourced
if [ -z "$ROS_DISTRO" ] || [ "$ROS_DISTRO" != "humble" ]; then
    echo "Error: ROS 2 Humble is not sourced. Please source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✓ ROS 2 Humble environment detected"

# Check essential ROS 2 packages
echo "Checking essential packages..."
if ! command -v colcon &> /dev/null; then
    echo "Error: colcon is not installed"
    exit 1
fi

if ! command -v check_urdf &> /dev/null; then
    echo "Error: check_urdf is not installed (ros-humble-urdf-tutorial package needed)"
    exit 1
fi

echo "✓ Essential packages are available"

# Create a test workspace to validate examples
TEST_WS="$HOME/module1_test_ws"
echo "Creating test workspace at $TEST_WS..."

mkdir -p $TEST_WS/src
cd $TEST_WS

# Create a test package
cd src
ros2 pkg create --build-type ament_python test_module1_examples
cd test_module1_examples/test_module1_examples

# Create example files from the module
cat > minimal_publisher.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

cat > minimal_subscriber.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "✓ Created example files for validation"

# Go back to workspace root and build
cd $TEST_WS
colcon build --packages-select test_module1_examples

if [ $? -eq 0 ]; then
    echo "✓ Code examples build successfully"
else
    echo "✗ Code examples failed to build"
    exit 1
fi

echo "✓ All code examples validated successfully"
```

## 2. URDF Validation Script

```python
#!/usr/bin/env python3
"""
URDF Validation Script for Module 1
Validates URDF files according to the examples in Chapter 5
"""

import subprocess
import sys
import os
import xml.etree.ElementTree as ET

def validate_urdf_file(urdf_path):
    """Validate a URDF file using ROS 2 tools"""
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file {urdf_path} does not exist")
        return False

    try:
        # Check URDF syntax
        print(f"Validating URDF file: {urdf_path}")
        result = subprocess.run(['check_urdf', urdf_path],
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"URDF validation failed: {result.stderr}")
            return False
        else:
            print("✓ URDF syntax is valid")
            return True
    except FileNotFoundError:
        print("Error: check_urdf command not found. Is ROS 2 installed?")
        return False
    except Exception as e:
        print(f"Error validating URDF: {e}")
        return False

def validate_humanoid_urdf(urdf_path):
    """Comprehensive validation for humanoid URDF files"""
    print(f"Validating humanoid URDF: {urdf_path}")

    # 1. Basic syntax validation
    print("\n1. Checking URDF syntax...")
    if not validate_urdf_file(urdf_path):
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
        print("   or: python3 validate_humanoid_urdf.py --example")
        if len(sys.argv) == 1:
            print("\nValidating the example humanoid URDF from Chapter 5...")
            # Create a simple example URDF for validation
            example_urdf = "example_humanoid.urdf"
            with open(example_urdf, 'w') as f:
                f.write('''<?xml version="1.0"?>
<robot name="example_humanoid">
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
  </joint>

  <link name="left_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.0 -0.05 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
  </joint>
</robot>''')
            success = validate_humanoid_urdf(example_urdf)
            os.remove(example_urdf)  # Clean up
            return success
        return False

    urdf_file = sys.argv[1]
    if not os.path.exists(urdf_file):
        print(f"Error: File {urdf_file} does not exist")
        return False

    success = validate_humanoid_urdf(urdf_file)
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
```

## 3. Cross-link Validation Script

```bash
#!/bin/bash
# validate_cross_links.sh - Check internal links in Module 1 documentation

echo "Validating cross-links in Module 1 documentation..."

DOCS_DIR="frontend-docu/docs/module-1-robotic-nervous-system"

# Check if the directory exists
if [ ! -d "$DOCS_DIR" ]; then
    echo "Error: Module 1 documentation directory not found at $DOCS_DIR"
    exit 1
fi

echo "Checking for broken internal links..."

# Find all markdown files in the module
find "$DOCS_DIR" -name "*.md" -type f | while read -r file; do
    # Extract links from the file
    grep -oE '\[.*\]\(([^)]+)\)' "$file" | grep -oE '\(([^)]+)\)' | sed 's/[()]//g' | while read -r link; do
        # Skip external links and anchors
        if [[ $link =~ ^https?:// ]] || [[ $link =~ ^# ]] || [[ $link =~ ^mailto: ]]; then
            continue
        fi

        # Convert relative links to absolute paths
        if [[ $link =~ ^\.\. ]]; then
            # Relative link from the file's directory
            dir=$(dirname "$file")
            abs_link="$dir/$link"
        elif [[ $link =~ ^/ ]]; then
            # Absolute link in docs directory
            abs_link="../..$link"
        else
            # Relative link in same directory
            dir=$(dirname "$file")
            abs_link="$dir/$link"
        fi

        # Remove anchors from the path
        clean_link=$(echo "$abs_link" | sed 's/#.*//')

        # Check if the file exists
        if [ ! -f "$clean_link" ]; then
            echo "✗ Broken link in $file: $link -> $clean_link does not exist"
        else
            echo "✓ Valid link in $file: $link"
        fi
    done
done

echo "Cross-link validation complete."
```

## 4. Readability Check Script

```python
#!/usr/bin/env python3
"""
Readability Analysis Script
Checks if the documentation meets Grade 10-12 reading level requirement
"""

import os
import re
from collections import Counter

def count_syllables(word):
    """Count syllables in a word"""
    word = word.lower()
    vowels = "aeiouy"
    syllable_count = 0
    prev_was_vowel = False

    for i, letter in enumerate(word):
        is_vowel = letter in vowels
        if is_vowel and not prev_was_vowel:
            syllable_count += 1
        prev_was_vowel = is_vowel

    # Handle silent 'e' at the end
    if word.endswith('e') and syllable_count > 1:
        syllable_count -= 1

    # Handle words that end with 'ed' but are pronounced as separate syllable
    if word.endswith('ed') and not word.endswith(('ted', 'ded')):
        syllable_count -= 1

    return max(1, syllable_count)

def flesch_reading_ease(total_words, total_sentences, total_syllables):
    """Calculate Flesch Reading Ease score"""
    if total_words == 0 or total_sentences == 0:
        return 0

    score = 206.835 - (1.015 * (total_words / total_sentences)) - (84.6 * (total_syllables / total_words))
    return score

def flesch_kincaid_grade_level(total_words, total_sentences, total_syllables):
    """Calculate Flesch-Kincaid Grade Level"""
    if total_words == 0 or total_sentences == 0:
        return 0

    grade_level = (0.39 * (total_words / total_sentences)) + (11.8 * (total_syllables / total_words)) - 15.59
    return grade_level

def analyze_readability(text):
    """Analyze readability of text"""
    # Split text into sentences
    sentences = re.split(r'[.!?]+', text)
    sentences = [s.strip() for s in sentences if s.strip()]

    # Split text into words
    words = re.findall(r'\b\w+\b', text.lower())
    words = [w for w in words if w]

    # Count syllables
    syllables = [count_syllables(word) for word in words]
    total_syllables = sum(syllables)

    # Calculate metrics
    total_words = len(words)
    total_sentences = len(sentences)
    total_syllables = sum(syllables)

    # Calculate readability scores
    ease_score = flesch_reading_ease(total_words, total_sentences, total_syllables)
    grade_level = flesch_kincaid_grade_level(total_words, total_sentences, total_syllables)

    return {
        'total_words': total_words,
        'total_sentences': total_sentences,
        'total_syllables': total_syllables,
        'flesch_reading_ease': ease_score,
        'flesch_kincaid_grade': grade_level
    }

def analyze_module_readability():
    """Analyze readability of Module 1 documentation"""
    docs_dir = "frontend-docu/docs/module-1-robotic-nervous-system"

    if not os.path.exists(docs_dir):
        print(f"Error: Directory {docs_dir} does not exist")
        return False

    all_text = ""

    # Read all markdown files in the module
    for filename in os.listdir(docs_dir):
        if filename.endswith('.md'):
            filepath = os.path.join(docs_dir, filename)
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
                # Remove markdown syntax for readability analysis
                content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)  # Remove code blocks
                content = re.sub(r'`[^`]*`', '', content)  # Remove inline code
                content = re.sub(r'^#+\s.*$', '', content, flags=re.MULTILINE)  # Remove headings
                content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)  # Remove links, keep text
                content = re.sub(r'\*\*([^*]+)\*\*', r'\1', content)  # Remove bold
                content = re.sub(r'\*([^*]+)\*', r'\1', content)  # Remove italic
                content = re.sub(r'^\s*[-*+]\s+', '', content, flags=re.MULTILINE)  # Remove list markers
                all_text += " " + content

    if not all_text.strip():
        print("Error: No readable text found in Module 1 documentation")
        return False

    # Analyze readability
    results = analyze_readability(all_text)

    print(f"Readability Analysis for Module 1:")
    print(f"  Total Words: {results['total_words']}")
    print(f"  Total Sentences: {results['total_sentences']}")
    print(f"  Total Syllables: {results['total_syllables']}")
    print(f"  Flesch Reading Ease: {results['flesch_reading_ease']:.2f}")
    print(f"  Flesch-Kincaid Grade Level: {results['flesch_kincaid_grade']:.2f}")

    # Check if grade level is within target range (10-12)
    grade_level = results['flesch_kincaid_grade']
    if 10 <= grade_level <= 12:
        print(f"  ✓ Grade level {grade_level:.2f} is within target range (10-12)")
        return True
    else:
        print(f"  ✗ Grade level {grade_level:.2f} is outside target range (10-12)")
        return False

if __name__ == "__main__":
    success = analyze_module_readability()
    exit(0 if success else 1)
```

These validation scripts help ensure that all code examples, URDF files, cross-links, and readability requirements are met for Module 1.