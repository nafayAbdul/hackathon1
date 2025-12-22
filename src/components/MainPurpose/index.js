import React, { useEffect, useRef } from 'react';
import useIntersectionObserver from '@site/src/utils/useIntersectionObserver';

const MainPurposeSection = () => {
  const { observe, unobserve } = useIntersectionObserver();
  const sectionRef = useRef(null);

  useEffect(() => {
    if (sectionRef.current) {
      observe(sectionRef.current);
    }

    return () => {
      if (sectionRef.current) {
        unobserve(sectionRef.current);
      }
    };
  }, [observe, unobserve]);

  return (
    <section ref={sectionRef} className="main-purpose-section">
      <div className="container">
        <div className="purpose-content">
          <div className="purpose-highlight">
            <div className="highlight-accent"></div>
            <h2 className="purpose-title">Our Mission</h2>
          </div>
          <p className="purpose-description">
            To advance the field of Physical AI and Humanoid Robotics by providing practitioners with the definitive knowledge base 
            and hands-on experience needed to build the next generation of intelligent machines that perceive, reason, and act in the physical world.
          </p>
          <div className="purpose-features">
            <div className="purpose-feature">
              <div className="feature-icon">
                <i className="fas fa-brain"></i>
              </div>
              <div className="feature-content">
                <h3>Embodied Intelligence</h3>
                <p>Understanding how AI systems can interact meaningfully with physical environments</p>
              </div>
            </div>
            <div className="purpose-feature">
              <div className="feature-icon">
                <i className="fas fa-robot"></i>
              </div>
              <div className="feature-content">
                <h3>Humanoid Control</h3>
                <p>Implementing advanced locomotion and manipulation techniques for human-like robots</p>
              </div>
            </div>
            <div className="purpose-feature">
              <div className="feature-icon">
                <i className="fas fa-cogs"></i>
              </div>
              <div className="feature-content">
                <h3>Production Systems</h3>
                <p>Building deployable AI systems for real-world robotics applications</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default MainPurposeSection;