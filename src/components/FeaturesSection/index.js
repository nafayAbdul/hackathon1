import React, { useEffect, useRef, useState } from 'react';
import useIntersectionObserver from '@site/src/utils/useIntersectionObserver';

const FeaturesSection = () => {
  const { observe, unobserve } = useIntersectionObserver();
  const sectionRef = useRef(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    if (sectionRef.current) {
      observe(sectionRef.current);
      // Set visible when section comes into view
      const handleIntersect = (entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            setIsVisible(true);
            unobserve(sectionRef.current);
          }
        });
      };

      const observer = new IntersectionObserver(handleIntersect, { threshold: 0.1 });
      observer.observe(sectionRef.current);

      return () => {
        if (sectionRef.current) {
          observer.unobserve(sectionRef.current);
        }
      };
    }

    return () => {
      if (sectionRef.current) {
        unobserve(sectionRef.current);
      }
    };
  }, [observe, unobserve]);

  const features = [
    {
      title: "Embodied Intelligence",
      description: "Design AI systems that understand physical constraints and interact meaningfully with the environment through sensory-motor loops.",
      icon: "fas fa-brain"
    },
    {
      title: "Humanoid Control",
      description: "Implement advanced locomotion and manipulation techniques for human-like robots using reinforcement learning and optimal control.",
      icon: "fas fa-robot"
    },
    {
      title: "Production Systems",
      description: "Build deployable AI systems with ROS 2, Docker containers, and edge computing architectures for real-world robotics applications.",
      icon: "fas fa-code"
    }
  ];

  return (
    <section id="features" ref={sectionRef} className={`features-section ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
      <div className="container">
        <div className={`section-title ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
          <h2>Core Learning Path</h2>
          <p>Master the full spectrum of physical AI development through practical, project-based learning</p>
        </div>
        <div className="features-grid">
          {features.map((feature, index) => (
            <div key={index} className={`feature-card ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`} style={{transitionDelay: `${index * 0.1}s`}}>
              <div className="feature-icon">
                <i className={feature.icon}></i>
              </div>
              <h3 className="feature-title">{feature.title}</h3>
              <p className="feature-description">{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default FeaturesSection;