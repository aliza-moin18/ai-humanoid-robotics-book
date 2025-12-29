import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Hands-On Capstone Textbook for Humanoid Robotics"
    >
      {/* Hero Section */}
      <header className={styles.hero}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            Physical AI & Humanoid Robotics
          </h1>
          <p className={styles.heroSubtitle}>
            From Simulated Brains to Embodied Intelligence
          </p>
          <p className={styles.heroDesc}>
            A Hands-On Capstone Textbook using ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action Models
          </p>
          <div className={styles.buttons}>
            <Link className={styles.primaryButton} to="/docs/module-1-ros2-fundamentals">
              Start Reading →
            </Link>
          </div>
        </div>
      </header>

      {/* Modules Overview Section */}
      <section className={styles.modulesSection}>
        <div className="container">
          <h2 className={styles.sectionTitle}>Explore the Modules</h2>
          <div className={styles.modulesGrid}>
            <div className={styles.moduleCard}>
              <h3>Module 1: ROS 2 Fundamentals</h3>
              <p>Master the robotic nervous system: nodes, topics, services, URDF, and simulation basics.</p>
              <Link to="/docs/module-1-ros2-fundamentals" className={styles.cardLink}>
                Start Module 1 →
              </Link>
            </div>

            <div className={styles.moduleCard}>
              <h3>Module 2: Digital Twin</h3>
              <p>Build high-fidelity simulations with Gazebo and Unity for sensor integration and visualization.</p>
              <Link to="/docs/module-2-digital-twin" className={styles.cardLink}>
                Start Module 2 →
              </Link>
            </div>

            <div className={styles.moduleCard}>
              <h3>Module 3: AI Robot Brain</h3>
              <p>Advanced AI training, sim-to-real transfer, and embodied intelligence deployment.</p>
              <Link to="/docs/module-3-ai-robot-brain" className={styles.cardLink}>
                Start Module 3 →
              </Link>
            </div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className={styles.featuresSection}>
        <div className="container">
          <h2 className={styles.sectionTitle}>Why This Book?</h2>
          <div className={styles.featuresGrid}>
            <div className={styles.featureItem}>
              <h3>Hands-On Approach</h3>
              <p>Real code examples, labs, and projects you can run yourself.</p>
            </div>
            <div className={styles.featureItem}>
              <h3>From Simulation to Real World</h3>
              <p>Learn sim-to-real transfer and deployment techniques.</p>
            </div>
            <div className={styles.featureItem}>
              <h3>Modern Tools</h3>
              <p>ROS 2, Gazebo, Isaac Sim, and cutting-edge AI models.</p>
            </div>
          </div>
        </div>
      </section>

    {/* Polished Footer */}
      <footer className={styles.footer}>
        <div className="container">
          <div className={styles.footerTop}>
            <div className={styles.footerLogo}>
              <img src="/img/robo.png" alt="AI Robotics Book" className={styles.logoImg} />
              <span>Physical AI & Humanoid Robotics</span>
            </div>

            <div className={styles.footerLinks}>
              <Link to="/">Home</Link>
              <Link to='/docs/module-1-ros2-fundamentals'>Modules 1</Link>
              <Link to='/docs/module-2-digital-twin/chapter-1/'>Module 2</Link>
              <Link to='/docs/module-3-ai-robot-brain/chapter-11/section-11-1-sim-to-real-transfer-techniques'>Module 3</Link>
            </div>
          </div>

          <p className={styles.copyright}>
            © {new Date().getFullYear()} Physical AI & Humanoid Robotics Book. All rights reserved.
          </p>
        </div>
      </footer>
    </Layout>
  );
}