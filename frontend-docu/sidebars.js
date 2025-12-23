// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'module-1-robotic-nervous-system/nervous-system-intro',
        'module-1-robotic-nervous-system/ros2-architecture-setup',
        'module-1-robotic-nervous-system/communication-patterns',
        'module-1-robotic-nervous-system/python-ai-bridge',
        'module-1-robotic-nervous-system/humanoid-kinematics-urdf',
        'module-1-robotic-nervous-system/summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      items: [
        'module-2/06-intro-to-digital-twins',
        'module-2/07-physics-in-gazebo',
        'module-2/08-high-fidelity-rendering-unity',
        'module-2/09-virtual-sensors-data',
        'module-2/10-sim2real-gap',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module-3/isaac-overview',
        'module-3/sdg-pipeline',
        'module-3/isaac-ros-vslam',
        'module-3/nav2-humanoid',
        'module-3/jetson-deployment',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/whisper-voice-control',
        'module-4/llm-task-planning',
        'module-4/vlm-object-identification',
        'module-4/capstone-system-design',
        'module-4/capstone-final-execution',
      ],
    },
  ]
};

export default sidebars;
