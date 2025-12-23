import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageModulesGrid from '@site/src/components/HomepageModulesGrid';
import styles from './index.module.css';

function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">
          {siteConfig.title}
        </h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with ROS 2
          </Link>
        </div>
      </div>
    </header>
  );
}

function CourseIntroduction() {
  return (
    <section className={styles.courseIntroduction}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h2 className={styles.courseIntroductionTitle}>
              Master <span className={styles.courseIntroductionHighlight}>Physical AI</span> & Humanoid Robotics
            </h2>
            <p className={styles.courseIntroductionDescription}>
              This comprehensive course is designed for AI students entering the exciting field of humanoid robotics.
              Learn to build intelligent robots that can perceive, navigate, and interact with the physical world using
              cutting-edge technologies like ROS 2, NVIDIA Isaac, and Vision-Language-Action systems.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CallToActionSection() {
  return (
    <section className={styles.callToActionSection}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h2 className={styles.callToActionTitle}>
              Ready to Start Your Robotics Journey?
            </h2>
            <p className={styles.callToActionDescription}>
              Begin with Module 1 to learn the fundamentals of ROS 2, or jump to any section that interests you most.
            </p>
            <div className={styles.callToActionButtons}>
              <Link
                className={clsx('button button--primary button--lg', styles.callToActionButton, styles.callToActionButtonPrimary)}
                to="/docs/module-1-ros2-fundamentals">
                Start with Module 1
              </Link>
              <Link
                className={clsx('button button--outline button--lg', styles.callToActionButton, styles.callToActionButtonSecondary)}
                to="/docs/module-1-ros2-fundamentals">
                Browse All Modules
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  const modules = [
    {
      title: 'ROS 2 Fundamentals',
      description: 'Learn the basics of ROS 2 as the robotic nervous system, including nodes, topics, services, and actions.',
      href: '/docs/module-1-ros2-fundamentals',
      moduleNumber: '1',
      category: 'Core Concepts',
      highlighted: true,
      image: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMjAwIDIwMCI+CiAgPHJlY3Qgd2lkdGg9IjIwMCIgaGVpZ2h0PSIyMDAiIGZpbGw9IiMwMDJhNTIiLz4KICA8Y2lyY2xlIGN4PSIxMDAiIGN5PSI2MCIgcj0iMjAiIGZpbGw9IiMwMGFiZGQiLz4KICA8Y2lyY2xlIGN4PSI2MCIgY3k9IjE0MCIgcj0iMTUiIGZpbGw9IiMwMGFiZGQiLz4KICA8Y2lyY2xlIGN4PSIxNDAiIGN5PSIxNDAiIHI9IjE1IiBmaWxsPSIjMDBhYmRkIi8+CiAgPHBhdGggZD0iTTgwIDEwMEgxMjAgTTEwMCA4MFYxMjAgTTEwMCAxMDBNODAgMTIwTDEyMCA4MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjIiLz4KICA8Y2lyY2xlIGN4PSIxMDAiIGN5PSIxMDAiIHI9IjEwIiBmaWxsPSIjZmZmIi8+Cjwvc3ZnPg=='
    },
    {
      title: 'ROS 2 Communication',
      description: 'Deep dive into nodes, topics, services, and data flow patterns for robust robotic communication.',
      href: '/docs/module-1-ros2-communication',
      moduleNumber: '2',
      category: 'Communication',
      highlighted: false,
      image: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMjAwIDIwMCI+CiAgPHJlY3Qgd2lkdGg9IjIwMCIgaGVpZ2h0PSIyMDAiIGZpbGw9IiMwMDI1NjQiLz4KICA8Y2lyY2xlIGN4PSI1MCIgY3k9IjUwIiByPSIyMCIgZmlsbD0iIzAwYmZmZiIvPgogIDxjaXJjbGUgY3g9IjE1MCIgY3k9IjUwIiByPSIyMCIgZmlsbD0iIzAwYmZmZiIvPgogIDxjaXJjbGUgY3g9IjUwIiBjeT0iMTUwIiByPSIyMCIgZmlsbD0iIzAwYmZmZiIvPgogIDxjaXJjbGUgY3g9IjE1MCIgY3k9IjE1MCIgcj0iMjAiIGZpbGw9IiMwMGJmZmYiLz4KICA8cGF0aCBkPSJNNzAgNTBIMTMwIE01MCA3MFYxMzAgTTcwIDE1MEgxMzAgTTE1MCA3MFYxMzAiIGZpbGw9Im5vbmUiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIyIi8+CiAgPHBhdGggZD0iTTgwIDgwTDEyMCAxMjAgTTEyMCA4MEw4MCAxMjAiIGZpbGw9Im5vbmUiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIyIi8+Cjwvc3ZnPg=='
    },
    {
      title: 'URDF Modeling',
      description: 'Create humanoid robot structure with links and joints using Unified Robot Description Format.',
      href: '/docs/module-1-urdf-modeling',
      moduleNumber: '3',
      category: 'Modeling',
      highlighted: false,
      image: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMjAwIDIwMCI+CiAgPHJlY3Qgd2lkdGg9IjIwMCIgaGVpZ2h0PSIyMDAiIGZpbGw9IiMwMDMwNjIiLz4KICA8Y2lyY2xlIGN4PSIxMDAiIGN5PSI0MCIgcj0iMTUiIGZpbGw9IiMwMGNjY2MiLz4KICA8Y2lyY2xlIGN4PSI2MCIgY3k9IjgwIiByPSIxMCIgZmlsbD0iIzAwY2NjYyIvPgogIDxjaXJjbGUgY3g9IjE0MCIgY3k9IjgwIiByPSIxMCIgZmlsbD0iIzAwY2NjYyIvPgogIDxjaXJjbGUgY3g9IjgwIiBjeT0iMTIwIiByPSIxMCIgZmlsbD0iIzAwY2NjYyIvPgogIDxjaXJjbGUgY3g9IjEyMCIgY3k9IjEyMCIgcj0iMTAiIGZpbGw9IiMwMGNjY2MiLz4KICA8Y2lyY2xlIGN4PSIxMDAgY3k9IjE2MCIgcj0iMTUiIGZpbGw9IiMwMGNjY2MiLz4KICA8cGF0aCBkPSJNMTAwIDU1Vjc1IE04MCA5NVYxMDUgTDEyMCA5NVYxMDUgTTgwIDEzNVYxNDUgTDEyMCAxMzV2MTAgTTg1IDgwTDEwMCA5NSBMMTE1IDgwIE04NSAxNjBMMTAwIDE0NSBMMTE1IDE2MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjIiLz4KPC9zdmc+'
    },
    {
      title: 'Digital Twin (Gazebo & Unity)',
      description: 'Build physics simulation and virtual environments for testing and validating your robots.',
      href: '/docs/module-2-digital-twin',
      moduleNumber: '4',
      category: 'Simulation',
      highlighted: true,
      image: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMjAwIDIwMCI+CiAgPHJlY3Qgd2lkdGg9IjIwMCIgaGVpZ2h0PSIyMDAiIGZpbGw9IiMwMDM1NzIiLz4KICA8cmVjdCB4PSI0MCIgeT0iNDAiIHdpZHRoPSIxMjAiIGhlaWdodD0iMTIwIiByeD0iOCIgcnk9IjgiIGZpbGw9Im5vbmUiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIzIi8+CiAgPGNpcmNsZSBjeD0iMTAwIiBjeT0iMTAwIiByPSIyMCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjIiLz4KICA8cGF0aCBkPSJNMTAwIDgwVjEyMCBNNjAgMTAwSDE0MCBNNjAgNjBIMTQwIE02MCAxNDBIMTQwIE02MCA2MEg2MCBNNjAgMTQwSDYwIE0xNDAgNjBIMTQwIE0xNDAgMTQwSDE0MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjEiLz4KICA8cmVjdCB4PSI4MCIgeT0iODAiIHdpZHRoPSI0MCIgaGVpZ2h0PSI0MCIgcng9IjIiIHJ5PSIyIiBmaWxsPSIjZmZmIi8+Cjwvc3ZnPg=='
    },
    {
      title: 'AI-Robot Brain (NVIDIA Isaac)',
      description: 'Implement perception, navigation, and synthetic data processing with NVIDIA Isaac platform.',
      href: '/docs/module-3-nvidia-isaac/nvidia-isaac-sim-fundamentals',
      moduleNumber: '5',
      category: 'AI',
      highlighted: false,
      image: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMjAwIDIwMCI+CiAgPHJlY3Qgd2lkdGg9IjIwMCIgaGVpZ2h0PSIyMDAiIGZpbGw9IiMwMDQwODAiLz4KICA8Y2lyY2xlIGN4PSIxMDAiIGN5PSIxMDAiIHI9IjQwIiBmaWxsPSJub25lIiBzdHJva2U9IiNmZmYiIHN0cm9rZS13aWR0aD0iMiIvPgogIDxjaXJjbGUgY3g9IjEwMCIgY3k9IjEwMCIgcj0iMjAiIGZpbGw9Im5vbmUiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIyIi8+CiAgPGNpcmNsZSBjeD0iMTAwIiBjeT0iMTAwIiByPSIxMCIgZmlsbD0iI2ZmZiIvPgogIDxwYXRoIGQ9Ik0xMjAgODAgTTE0MCA2MCBNNjAgMTQwIE04MCAxNjAgTTEyMCAxMjAgTTE0MCAxNDAgTTgwNjAgTTgwIDE0MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjEiLz4KICA8cGF0aCBkPSJNMTAwIDYwTDEyMCA4MCBNNjAgMTIwTDgwIDE0MCBNNjAgODBMODAgNjAgTTIwIDEwMEg0MCBNNjAgMTYwTDgwIDE4MCBNOTYgMTgwTDEwNCAxODAgTTE2MCAxMDBMMTgwIDEyMCBNOTYgNjBMMTA0IDYwIiBmaWxsPSJub25lIiBzdHJva2U9IiNmZmYiIHN0cm9rZS13aWR0aD0iMSIvPgogIDxwYXRoIGQ9Ik04MCAxMDBMMTAwIDEyMCBMMTIwIDEwMCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjEiLz4KPC9zdmc+'
    },
    {
      title: 'Vision-Language-Action (VLA)',
      description: 'Integrate voice, LLM planning, and autonomous actions for intelligent robot behavior.',
      href: '/docs/module-4-vla/capstone-autonomous-humanoid',
      moduleNumber: '6',
      category: 'Intelligence',
      highlighted: true,
      image: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMjAwIDIwMCI+CiAgPHJlY3Qgd2lkdGg9IjIwMCIgaGVpZ2h0PSIyMDAiIGZpbGw9IiMwMDUwYjAiLz4KICA8Y2lyY2xlIGN4PSIxMDAiIGN5PSI1MCIgcj0iMTUiIGZpbGw9IiNmZmYiLz4KICA8cGF0aCBkPSJNMTAwIDY1VDEwNSBNNjAgMTAwTDgwIDEyMCBMMTAwIDEwMCBMMTIwIDEyMCBMMTYwIDEwMCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjIiLz4KICA8cGF0aCBkPSJNNTAgMTUwTDcwIDEzMCBNOTAgMTUwIE0xMTAgMTUwTDEzMCAxMzAgTDE1MCAxNTAiIGZpbGw9Im5vbmUiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIyIi8+CiAgPHBhdGggZD0iTTgwIDE1MFYxNjAgTTEyMCAxNTBWMjAwIE0xMDAgMTMwTDEwMCAxNTAgTTEwMCAxNzBWMjAwIiBmaWxsPSJub25lIiBzdHJva2U9IiNmZmYiIHN0cm9rZS13aWR0aD0iMiIvPgogIDxwYXRoIGQ9Ik0xMDAgMTUwTTEyMCAxNzBMMTAwIDE5MCBNOCAxNzBMMTAwIDE1MCBNOCAxMzBMMTAwIDE1MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjIiLz4KPC9zdmc+'
    }
  ];

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics course - Learn ROS 2, NVIDIA Isaac, and Vision-Language-Action systems">
      <HomepageHero />
      <main>
        <CourseIntroduction />
        <HomepageModulesGrid
          title="Learning Modules"
          description="Explore our comprehensive modules designed for AI students entering humanoid robotics"
          modules={modules}
        />
        <CallToActionSection />
      </main>
    </Layout>
  );
}
