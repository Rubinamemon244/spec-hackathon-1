import React from 'react';
import Layout from '@theme/Layout';
import RoboticsThemeSwitcher from '../components/RoboticsThemeSwitcher';

function RoboticsThemes() {
  return (
    <Layout title="Robotics Themes" description="AI/Robotics Color Themes for ROS 2 Education">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>AI/Robotics Color Themes</h1>
            <p>
              Welcome to the AI/Robotics-themed color options for the ROS 2 for Humanoid Robotics Education platform. Choose from three futuristic, technical themes designed for developers and robotics enthusiasts.
            </p>

            <div className="margin-vert--lg" style={{
              backgroundColor: 'var(--ifm-background-surface-color)',
              padding: '1.5rem',
              borderRadius: '0.5rem'
            }}>
              <h2>Select Theme</h2>
              <RoboticsThemeSwitcher />
              <p style={{ marginTop: '1rem' }}>
                Choose any of the themes above to see it applied to the entire site. Your selection will be remembered across page visits.
              </p>
            </div>

            <h2>Available Themes</h2>

            <div className="margin-vert--md" style={{
              backgroundColor: 'var(--ifm-background-surface-color)',
              padding: '1.5rem',
              borderRadius: '0.5rem',
              borderLeft: '4px solid #00eeff'
            }}>
              <h3>1. Dark AI Lab</h3>
              <ul>
                <li><strong>Base</strong>: Near-black / charcoal background</li>
                <li><strong>Accents</strong>: Electric blue and cyan colors</li>
                <li><strong>Style</strong>: Futuristic AI laboratory aesthetic</li>
              </ul>
            </div>

            <div className="margin-vert--md" style={{
              backgroundColor: 'var(--ifm-background-surface-color)',
              padding: '1.5rem',
              borderRadius: '0.5rem',
              borderLeft: '4px solid #00ff9d'
            }}>
              <h3>2. Robotics Steel</h3>
              <ul>
                <li><strong>Base</strong>: Cool gray background</li>
                <li><strong>Accents</strong>: Neon green and teal colors</li>
                <li><strong>Style</strong>: Industrial robotics aesthetic</li>
              </ul>
            </div>

            <div className="margin-vert--md" style={{
              backgroundColor: 'var(--ifm-background-surface-color)',
              padding: '1.5rem',
              borderRadius: '0.5rem',
              borderLeft: '4px solid #9d4edd'
            }}>
              <h3>3. Neural Horizon</h3>
              <ul>
                <li><strong>Base</strong>: Midnight blue background</li>
                <li><strong>Accents</strong>: Purple and soft orange colors</li>
                <li><strong>Style</strong>: Neural network and AI horizon aesthetic</li>
              </ul>
            </div>

            <div className="margin-vert--md" style={{
              backgroundColor: 'var(--ifm-background-surface-color)',
              padding: '1.5rem',
              borderRadius: '0.5rem',
              borderLeft: '4px solid #ff00ff'
            }}>
              <h3>4. Quantum Cyberpunk</h3>
              <ul>
                <li><strong>Base</strong>: Deep space black background</li>
                <li><strong>Accents</strong>: Vibrant neon pink/purple and electric lime green</li>
                <li><strong>Style</strong>: Cyberpunk and quantum computing aesthetic</li>
              </ul>
            </div>

            <div className="margin-vert--md" style={{
              backgroundColor: 'var(--ifm-background-surface-color)',
              padding: '1.5rem',
              borderRadius: '0.5rem',
              borderLeft: '4px solid #b87333'
            }}>
              <h3>5. Biomechanical</h3>
              <ul>
                <li><strong>Base</strong>: Organic dark gray background</li>
                <li><strong>Accents</strong>: Metallic copper/bronze and bio-neon green</li>
                <li><strong>Style</strong>: Biomechanical and organic tech aesthetic</li>
              </ul>
            </div>

            <div className="margin-vert--lg" style={{
              backgroundColor: 'var(--ifm-color-emphasis-100)',
              padding: '1rem',
              borderRadius: '0.5rem',
              borderLeft: '4px solid var(--ifm-color-primary)'
            }}>
              <h3>💡 Pro Tip</h3>
              <p>
                Each theme is carefully designed to maintain readability and accessibility standards while providing a unique, robotics-inspired aesthetic. The themes work alongside Docusaurus's built-in dark/light mode.
              </p>
            </div>

            <h2>Theme Features</h2>
            <ul>
              <li>High contrast for improved readability</li>
              <li>Accessibility-focused design</li>
              <li>Responsive layouts</li>
              <li>Support for reduced motion and high contrast modes</li>
              <li>Modern, clean user interface</li>
            </ul>

            <h2>About the Physical AI & Humanoid Robotics Course</h2>
            <p>
              This theme selection is part of the <strong>Physical AI & Humanoid Robotics</strong> educational platform, designed to provide developers and students with comprehensive learning materials for ROS 2, humanoid robotics, and AI integration.
            </p>
            <p><strong>Instructor</strong>: Rubina Memon</p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default RoboticsThemes;