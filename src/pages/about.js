import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import './css/about-us.css';

const About = () => {
  useEffect(() => {
    // Function to handle scroll animations
    const handleScrollAnimations = () => {
      const elements = document.querySelectorAll('.section-content, .technologies-grid, .contact-buttons');

      elements.forEach(element => {
        const elementTop = element.getBoundingClientRect().top;
        const elementVisible = 150; // Pixel from bottom of screen

        if (elementTop < window.innerHeight - elementVisible) {
          element.classList.add('visible');
        }
      });
    };

    // Initial check on page load
    handleScrollAnimations();

    // Add scroll event listener
    window.addEventListener('scroll', handleScrollAnimations);

    // Cleanup event listener on component unmount
    return () => {
      window.removeEventListener('scroll', handleScrollAnimations);
    };
  }, []);

  return (
    <Layout title="About Us" description="Learn more about Physical AI & Humanoid Robotics">
      <div className="about-us-page">
        {/* Animated background particles */}
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>

        {/* Floating background elements for visual interest */}
        <div className="floating-element floating-element-1"></div>
        <div className="floating-element floating-element-2"></div>

        {/* Hero Section */}
        <section className="about-hero">
          <div className="container">
            <div className="hero-content">
              <h1 className="hero-title">About Physical AI & Humanoid Robotics</h1>
              <p className="hero-subtitle">
                Pioneering the future of embodied intelligence and robotics
              </p>
            </div>
          </div>
        </section>

        {/* Introduction Section */}
        <section className="intro-section">
          <div className="container">
            <div className="section-content">
              <h2>Revolutionizing the Future of Robotics</h2>
              <p>
                Physical AI represents a paradigm shift in artificial intelligence - moving beyond digital computation
                to systems that understand and interact with the physical world. Our mission is to create the next
                generation of intelligent machines that can perceive, reason, and act in complex real-world environments.
              </p>
              <p>
                Through our comprehensive guide and resources, we're preparing engineers, researchers, and
                practitioners for the era of embodied intelligence. From simulation to deployment, from control
                theory to deep learning, we cover the full spectrum of technologies required to build the robots
                of tomorrow.
              </p>
            </div>
          </div>
        </section>

        {/* Mission Section */}
        <section className="mission-section">
          <div className="container">
            <div className="section-content">
              <h2>Our Mission</h2>
              <p>
                We are dedicated to advancing the field of Physical AI - the intersection of artificial intelligence
                and physical systems. Our goal is to bridge the gap between digital intelligence and the physical world,
                enabling robots and AI systems that can perceive, reason, and act in complex real-world environments.
              </p>
              <p>
                We believe that the future of AI lies not in abstract algorithms alone, but in systems that can
                interact with the physical world through sensory-motor loops. This requires a deep understanding
                of physics, control theory, perception, and cognition working in harmony.
              </p>
            </div>
          </div>
        </section>

        {/* What is Physical AI Section */}
        <section className="what-is-physical-ai">
          <div className="container">
            <div className="section-content">
              <h2>What is Physical AI?</h2>
              <p>
                Physical AI is an interdisciplinary field that combines artificial intelligence, robotics,
                computer vision, machine learning, and control theory to create systems that can interact
                intelligently with the physical world. Unlike traditional AI that operates primarily in digital
                spaces, Physical AI systems must understand and navigate the complexities of real-world physics,
                uncertainty, and dynamics.
              </p>
              <p>
                This includes everything from:
              </p>
              <ul>
                <li>Sensor fusion and perception systems</li>
                <li>Real-time control algorithms for robotic systems</li>
                <li>Learning from physical interaction and experience</li>
                <li>Simulation-to-reality transfer techniques</li>
                <li>Human-robot interaction and collaboration</li>
                <li>Safety and ethical considerations in embodied AI</li>
              </ul>
            </div>
          </div>
        </section>

        {/* Team Section */}
        <section className="team-section">
          <div className="container">
            <div className="section-content">
              <h2>Our Team</h2>
              <p>
                Our team consists of leading researchers, engineers, and practitioners from top universities and
                technology companies who are passionate about pushing the boundaries of what's possible in robotics
                and embodied AI. Our diverse expertise spans across:
              </p>
              <ul>
                <li>Deep reinforcement learning and control theory</li>
                <li>Computer vision and sensor fusion</li>
                <li>Robotics hardware and design</li>
                <li>Simulation environments and physics engines</li>
                <li>Humanoid robot development</li>
                <li>Industrial automation and deployment</li>
              </ul>
              <p>
                We bring together academic rigor and practical application experience to provide comprehensive
                insights into the field of Physical AI and humanoid robotics.
              </p>
            </div>
          </div>
        </section>

        {/* Vision Section */}
        <section className="vision-section">
          <div className="container">
            <div className="section-content">
              <h2>Our Vision</h2>
              <p>
                We envision a future where intelligent physical systems seamlessly integrate into our daily lives,
                enhancing human capabilities and solving complex real-world challenges. From household robots to
                industrial automation, from healthcare assistants to space exploration, we're laying the groundwork
                for the next generation of AI-powered machines.
              </p>
              <p>
                Our vision extends beyond just building robots that can perform tasks - we aim to create systems
                that understand context, adapt to changing environments, collaborate with humans, and operate
                safely in complex, unstructured environments.
              </p>
            </div>
          </div>
        </section>

        {/* Approach Section */}
        <section className="approach-section">
          <div className="container">
            <div className="section-content">
              <h2>Our Approach</h2>
              <p>
                We take a holistic approach to teaching Physical AI, combining:
              </p>
              <ul>
                <li><strong>Theoretical Foundations:</strong> Deep understanding of the mathematical and physical principles underlying embodied intelligence</li>
                <li><strong>Hands-on Implementation:</strong> Practical coding exercises and projects that bring concepts to life</li>
                <li><strong>State-of-the-art Tools:</strong> Coverage of the latest frameworks, simulators, and platforms in the field</li>
                <li><strong>Real-world Applications:</strong> Case studies and examples from actual deployment scenarios</li>
                <li><strong>Future Outlook:</strong> Insights into emerging trends and technologies that will shape the future</li>
              </ul>
            </div>
          </div>
        </section>

        {/* Technologies Section */}
        <section className="technologies-section">
          <div className="container">
            <div className="section-content">
              <h2>Technologies We Cover</h2>
              <div className="technologies-grid">
                <div className="tech-item" style={{'--index': 1}}>
                  <h3>Simulation Environments</h3>
                  <p>Isaac Sim, Gazebo, PyBullet, Mujoco</p>
                </div>
                <div className="tech-item" style={{'--index': 2}}>
                  <h3>Robotics Frameworks</h3>
                  <p>ROS 2, PyTorch, TensorFlow, JAX</p>
                </div>
                <div className="tech-item" style={{'--index': 3}}>
                  <h3>Control Systems</h3>
                  <p>Optimal control, Model Predictive Control, Reinforcement Learning</p>
                </div>
                <div className="tech-item" style={{'--index': 4}}>
                  <h3>Perception</h3>
                  <p>Computer vision, LiDAR processing, Sensor fusion</p>
                </div>
                <div className="tech-item" style={{'--index': 5}}>
                  <h3>Humanoid Platforms</h3>
                  <p>Atlas, Digit, HRP-4, NAO, AMI</p>
                </div>
                <div className="tech-item" style={{'--index': 6}}>
                  <h3>Deployment</h3>
                  <p>Edge computing, Real-time systems, Safety protocols</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Contact Section */}
        <section className="contact-section">
          <div className="container">
            <div className="section-content">
              <h2>Get In Touch</h2>
              <p>
                Interested in learning more about our work, collaborating on research, or joining our community?
                Reach out to our team. We're always excited to connect with like-minded researchers, engineers,
                and enthusiasts who share our passion for advancing Physical AI and humanoid robotics.
              </p>
              <div className="contact-buttons">
                <a href="mailto:info@physicalai.com" className="cta-button">Contact Us</a>
                <a href="/docs" className="cta-button cta-button--secondary">Documentation</a>
              </div>
            </div>
          </div>
        </section>
      </div>
    </Layout>
  );
};

export default About;