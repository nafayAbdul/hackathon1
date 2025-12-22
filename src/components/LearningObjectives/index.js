import React, { useState, useEffect, useRef } from 'react';

const LearningObjectivesSection = () => {
  const [isVisible, setIsVisible] = useState(false);
  const sectionRef = useRef(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.disconnect(); // Only trigger once
        }
      },
      { threshold: 0.1 } // Trigger when 10% of the element is visible
    );

    if (sectionRef.current) {
      observer.observe(sectionRef.current);
    }

    return () => {
      if (sectionRef.current) {
        observer.unobserve(sectionRef.current);
      }
    };
  }, []);

  const objectives = [
    {
      title: "Embodied Cognition",
      description: "Learn how to design AI systems that understand physical constraints and interact meaningfully with the environment through sensory-motor loops."
    },
    {
      title: "Sensor Fusion",
      description: "Master the integration of multiple sensor modalities for robust perception in dynamic environments."
    },
    {
      title: "RL for Robotics",
      description: "Implement reinforcement learning techniques for developing adaptive robotic behaviors."
    }
  ];

  return (
    <section id="objectives" ref={sectionRef} className={`learning-objectives-section ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
      <div className="container">
        <div className={`section-title ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
          <h2>Learning Objectives</h2>
          <p>Understand the core concepts that enable robots to perceive, reason, and act in the physical world</p>
        </div>
        <div className="objectives-grid">
          {objectives.map((objective, index) => (
            <div key={index} className={`objective-card ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`} style={{transitionDelay: `${index * 0.1}s`}}>
              <div className="objective-border-accent"></div>
              <h3 className="objective-title">{objective.title}</h3>
              <p className="objective-description">{objective.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default LearningObjectivesSection;