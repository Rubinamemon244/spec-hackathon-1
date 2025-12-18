// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2-fundamentals/index',
        'module-1-ros2-fundamentals/architecture',
        'module-1-ros2-fundamentals/embodied-intelligence',
        'module-1-ros2-fundamentals/python-examples',
        'module-1-ros2-fundamentals/learning-objectives',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Communication',
      items: [
        'module-1-ros2-communication/index',
        'module-1-ros2-communication/nodes-topics-services',
        'module-1-ros2-communication/python-rclpy',
        'module-1-ros2-communication/communication-examples',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: URDF Modeling',
      items: [
        'module-1-urdf-modeling/index',
        'module-1-urdf-modeling/links-joints-frames',
        'module-1-urdf-modeling/hardware-mapping',
        'module-1-urdf-modeling/urdf-examples',
        'module-1-urdf-modeling/urdf-diagrams',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/physics-simulation-gazebo',
        'module-2-digital-twin/unity-environments',
        {
          type: 'category',
          label: 'Sensor Simulation',
          items: [
            'module-2-digital-twin/sensor-simulation',
            'module-2-digital-twin/lidar-simulation',
            'module-2-digital-twin/depth-camera-simulation',
            'module-2-digital-twin/imu-simulation',
          ],
        },
        'module-2-digital-twin/sensor-learning-objectives',
        'module-2-digital-twin/module-conclusion',
      ],
    },
  ],
};

export default sidebars;