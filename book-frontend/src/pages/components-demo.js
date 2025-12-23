import React from 'react';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';
import ModulesGrid from '@site/src/components/ModulesGrid';
import SectionHeader from '@site/src/components/SectionHeader';

function ComponentsDemo() {
  const heroButtons = [
    {
      text: 'Get Started',
      href: '/docs/module-1-ros2-fundamentals',
      external: false
    },
    {
      text: 'View Modules',
      href: '/docs/module-1-ros2-fundamentals',
      external: false
    }
  ];

  const modules = [
    {
      title: 'ROS 2 Fundamentals',
      description: 'Learn the basics of ROS 2, including nodes, topics, services, and actions. Understand the architecture and core concepts.',
      href: '/docs/module-1-ros2-fundamentals',
      category: 'Core Concepts',
      level: 'Beginner',
      tags: ['ROS 2', 'Nodes', 'Topics'],
      icon: '🤖',
      highlighted: true,
      variant: 'primary'
    },
    {
      title: 'ROS 2 Communication',
      description: 'Deep dive into ROS 2 communication patterns, message passing, and Quality of Service (QoS) settings.',
      href: '/docs/module-1-ros2-communication',
      category: 'Communication',
      level: 'Intermediate',
      tags: ['Topics', 'Services', 'Actions'],
      icon: '📡',
      variant: 'secondary'
    },
    {
      title: 'URDF Modeling',
      description: 'Create robot models using Unified Robot Description Format (URDF) for simulation and real-world applications.',
      href: '/docs/module-1-urdf-modeling',
      category: 'Modeling',
      level: 'Intermediate',
      tags: ['URDF', 'Robot Models', 'Simulation'],
      icon: '⚙️',
      highlighted: true
    },
    {
      title: 'Digital Twin (Gazebo & Unity)',
      description: 'Build and simulate digital twins of robots using Gazebo and Unity for advanced robotics applications.',
      href: '/docs/module-2-digital-twin',
      category: 'Simulation',
      level: 'Advanced',
      tags: ['Gazebo', 'Unity', 'Simulation'],
      icon: '🎮',
      variant: 'primary'
    },
    {
      title: 'AI-Robot Brain (NVIDIA Isaac)',
      description: 'Implement AI capabilities for robots using NVIDIA Isaac Sim and AI frameworks for intelligent behavior.',
      href: '/docs/module-3-nvidia-isaac/nvidia-isaac-sim-fundamentals',
      category: 'AI',
      level: 'Advanced',
      tags: ['AI', 'NVIDIA Isaac', 'Machine Learning'],
      icon: '🧠',
      variant: 'secondary'
    },
    {
      title: 'Robot Navigation',
      description: 'Learn about robot navigation systems, path planning, and autonomous movement in dynamic environments.',
      href: '/docs/module-1-ros2-fundamentals',
      category: 'Navigation',
      level: 'Advanced',
      tags: ['Navigation', 'Path Planning', 'Localization'],
      icon: '🧭',
      highlighted: true
    }
  ];

  return (
    <Layout title="UI Components Demo" description="Demo of reusable Docusaurus UI components">
      <HeroSection
        title="ROS 2 for Humanoid Robotics Education"
        subtitle="Modern, responsive UI components for your robotics course website"
        backgroundColor="primary"
        textColor="light"
        buttons={heroButtons}
        centered={true}
      />

      <main className="container margin-vert--lg">
        <SectionHeader
          title="Learning Modules"
          subtitle="Course Content"
          description="Explore our comprehensive modules designed for AI students entering humanoid robotics"
          align="center"
          size="large"
          divider={true}
          withIcon={true}
          icon="📚"
          iconPosition="top"
        />

        <ModulesGrid
          modules={modules}
          gridSize="medium"
          layout="grid"
          showFilter={true}
          title="Available Modules"
          description="Browse our collection of robotics learning modules"
        />

        <SectionHeader
          title="Featured Content"
          subtitle="Highlighted Resources"
          description="Our most popular and recommended learning materials"
          align="left"
          size="medium"
          divider={true}
          variant="primary"
        />

        <ModulesGrid
          modules={modules.filter(module => module.highlighted)}
          gridSize="large"
          layout="grid"
          showFilter={false}
        />
      </main>
    </Layout>
  );
}

export default ComponentsDemo;