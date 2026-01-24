import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Hands-On Capstone Textbook for Physical AI and Robotics"
    >

<section className="hero">
  <div className="heroContent">
    <h1 className="heroTitle">Physical AI & Humanoid Robotics</h1>
    <p className="heroSubtitle">
      Engineering-first textbook for building intelligent robotic systems
    </p>
    <p className="heroDesc">
      Learn, simulate, and implement AI-powered humanoid robots with hands-on examples and modules.
    </p>
    <a href="#modules" className="heroButton">Start the Reading</a>
  </div>

  <div className="heroImages">
    <img
      src="/img/new.png"
      alt="Robot Right"
      className={styles.heroImageRight} 
     />
  </div>
</section>


      {/* MODULES */}
      <section className={styles.section}>
        <h2 className={styles.sectionTitle}>Book Modules</h2>
        <div className={styles.grid}>
          <div className={styles.card}>
            <h3>ROS 2 Fundamentals</h3>
            <p>
              Nodes, topics, services, robot description, and simulation fundamentals.
            </p>
            <Link to="/docs/module-1-ros2-fundamentals">Read Module →</Link>
          </div>

          <div className={styles.card}>
            <h3>Digital Twins</h3>
            <p>
              Build realistic simulation environments using Gazebo and Isaac Sim.
            </p>
            <Link to="/docs/module-2-digital-twin/chapter-1/">Read Module →</Link>
          </div>

          <div className={styles.card}>
            <h3>AI Robot Brain</h3>
            <p>
              Train embodied AI models and deploy them into real robotic systems.
            </p>
            <Link to="/docs/module-3-ai-robot-brain/chapter-11/section-11-1-sim-to-real-transfer-techniques">Read Module →</Link>
          </div>
        </div>
      </section>

      {/* WHY */}
      <section className={`${styles.section} ${styles.sectionGray}`}>
        <h2 className={styles.sectionTitle}>Why This Book?</h2>
        <div className={styles.grid}>
          <div className={styles.card}>
            <h3>Engineering First</h3>
            <p>Focused on systems, not hype. Real tools, real workflows.</p>
          </div>
          <div className={styles.card}>
            <h3>Sim to Real</h3>
            <p>Learn how to move from simulation into physical robots.</p>
          </div>
          <div className={styles.card}>
            <h3>Modern Stack</h3>
            <p>ROS 2, Isaac Sim, Gazebo, and AI-based robot control.</p>
          </div>
        </div>
      </section>

      {/* ================= PROFESSIONAL FOOTER ================= */}
<footer className={styles.footer}>

  {/* BOTTOM BAR */}
  <div className={styles.footerBottom}>
    © {new Date().getFullYear()} Aliza Moin · Physical AI & Humanoid Robotics
  </div>
</footer>

    </Layout>
  );
}