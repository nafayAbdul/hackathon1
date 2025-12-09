import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary heroBanner_terminal')}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className="terminal-grid">
          <a href="/docs/module1/intro" className="terminal-card card-1">
            <div className="terminal-header">
              <div className="terminal-dots">
                <span className="dot red"></span>
                <span className="dot yellow"></span>
                <span className="dot green"></span>
              </div>
              <div className="terminal-title">module1.md</div>
            </div>
            <div className="terminal-body">
              <div className="terminal-content">
                <span className="module-number">01</span>
                <div className="terminal-prompt">$ cat intro</div>
                <h3>The Robotic Nervous System</h3>
                <p>Understanding the fundamentals of physical intelligence and embodied AI</p>
              </div>
            </div>
          </a>

          <a href="/docs/module2/intro" className="terminal-card card-2">
            <div className="terminal-header">
              <div className="terminal-dots">
                <span className="dot red"></span>
                <span className="dot yellow"></span>
                <span className="dot green"></span>
              </div>
              <div className="terminal-title">module2.md</div>
            </div>
            <div className="terminal-body">
              <div className="terminal-content">
                <span className="module-number">02</span>
                <div className="terminal-prompt">$ cat intro</div>
                <h3>Digital Twin & Simulation</h3>
                <p>Building accurate simulation environments for robot training</p>
              </div>
            </div>
          </a>

          <a href="/docs/module3/intro" className="terminal-card card-3">
            <div className="terminal-header">
              <div className="terminal-dots">
                <span className="dot red"></span>
                <span className="dot yellow"></span>
                <span className="dot green"></span>
              </div>
              <div className="terminal-title">module3.md</div>
            </div>
            <div className="terminal-body">
              <div className="terminal-content">
                <span className="module-number">03</span>
                <div className="terminal-prompt">$ cat intro</div>
                <h3>Simulation & Reinforcement Learning</h3>
                <p>Training intelligent agents in virtual environments</p>
              </div>
            </div>
          </a>

          <a href="/docs/module4/intro" className="terminal-card card-4">
            <div className="terminal-header">
              <div className="terminal-dots">
                <span className="dot red"></span>
                <span className="dot yellow"></span>
                <span className="dot green"></span>
              </div>
              <div className="terminal-title">module4.md</div>
            </div>
            <div className="terminal-body">
              <div className="terminal-content">
                <span className="module-number">04</span>
                <div className="terminal-prompt">$ cat intro</div>
                <h3>Vision-Language-Action Models</h3>
                <p>Integrating perception, decision-making, and action in robotics</p>
              </div>
            </div>
          </a>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="The definitive 2025 practitioner's book on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <div className="text--center padding-horiz--md">
                  <h2>Physical AI & Humanoid Robotics - The 2025 Practitioner's Guide</h2>
                  <p className="book-overview">
                    This comprehensive guide introduces you to the revolutionary field of Physical AI, where artificial intelligence meets real-world interaction.
                    Unlike traditional AI that operates in digital spaces, Physical AI focuses on embodied intelligence - systems that learn through physical interaction
                    with the world around them. This book is designed for engineers, researchers, and robotics enthusiasts looking to understand and build the next
                    generation of intelligent robotic systems.
                  </p>
                  <h3>Main Purpose</h3>
                  <p className="book-overview">
                    The main purpose of this book is to bridge the gap between theoretical AI knowledge and practical robotic applications. We aim to provide
                    readers with both the conceptual understanding and hands-on skills necessary to develop embodied AI systems that can interact intelligently
                    with the physical world. This book serves as a comprehensive resource for understanding how robots can exhibit true intelligence through
                    their interaction with the environment, rather than just processing abstract data.
                  </p>
                  <h3>Learning Objectives</h3>
                  <p className="book-overview">
                    By completing this book, you will be able to:
                  </p>
                  <ul className="book-objectives">
                    <li>Understand the fundamental differences between digital AI and Physical AI systems</li>
                    <li>Implement sensory-motor control systems for robotic platforms</li>
                    <li>Build and train reinforcement learning models specifically for robotic tasks</li>
                    <li>Develop simulation environments that accurately reflect real-world physics</li>
                    <li>Integrate multimodal perception (vision, language, and action) in robotic systems</li>
                    <li>Design control systems for complex humanoid robots with multiple degrees of freedom</li>
                    <li>Apply Vision-Language-Action (VLA) models to real robotic platforms</li>
                    <li>Implement safety and ethical considerations in autonomous robotic systems</li>
                    <li>Deploy embodied AI systems in real-world environments</li>
                    <li>Understand the current state and future directions of Physical AI research</li>
                  </ul>
                  <p className="book-overview">
                    You'll start by exploring the fundamentals of embodied intelligence and the core principles that differentiate Physical AI from classical
                    digital AI systems. Through practical examples and hands-on exercises, you'll gain insights into how robots can learn to navigate, manipulate
                    objects, and make decisions based on sensory feedback from the physical environment.
                  </p>
                  <p className="book-overview">
                    The book covers essential technologies including Robot Operating System 2 (ROS 2), simulation environments, reinforcement learning for
                    robotics, and the latest in Vision-Language-Action (VLA) models that are transforming how robots perceive and interact with the world.
                    Throughout the text, we use the "athena" 23-DoF humanoid robot as a practical platform to demonstrate concepts and implement real-world
                    applications.
                  </p>
                  <p className="book-overview">
                    Each module builds on the previous one, progressing from basic concepts to advanced applications. From understanding the "robotic nervous
                    system" that connects perception and action, to creating digital twins for simulation-based learning, to implementing cutting-edge VLA
                    models for sophisticated human-robot interaction, this book provides a complete roadmap for mastering Physical AI and Humanoid Robotics.
                  </p>
                  <p className="book-overview">
                    By the end of this book, you'll have the knowledge and practical skills to design, develop, and deploy advanced robotic systems that
                    exhibit true embodied intelligence, opening up opportunities in research, industry, and the rapidly expanding field of service robotics.
                  </p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}