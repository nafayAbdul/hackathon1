import React, { useState, useEffect, useRef } from 'react';

const TechStackSection = () => {
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

  const technologies = [
    {
      title: "PyTorch & CUDA",
      description: "Accelerated deep learning for real-time perception and control on edge devices",
      icon: "fas fa-microchip"
    },
    {
      title: "Isaac Sim",
      description: "High-fidelity physics simulation for training and validating robotic behaviors",
      icon: "fas fa-cube"
    },
    {
      title: "ROS 2",
      description: "Modular robotics middleware for building distributed robotic systems",
      icon: "fas fa-network-wired"
    },
    {
      title: "AWS RoboMaker",
      description: "Cloud infrastructure for large-scale robotic fleet management and simulation",
      icon: "fas fa-cloud"
    }
  ];

  return (
    <section id="technology" ref={sectionRef} className={`tech-stack ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
      <div className="container">
        <div className={`section-title ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
          <h2>Technology Stack</h2>
          <p>Master the tools and frameworks powering the next generation of physical AI systems</p>
        </div>
        <div className="tech-grid">
          {technologies.map((tech, index) => (
            <div key={index} className={`tech-item ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`} style={{transitionDelay: `${index * 0.1}s`}}>
              <div className="tech-icon">
                <i className={tech.icon}></i>
              </div>
              <h3>{tech.title}</h3>
              <p>{tech.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default TechStackSection;