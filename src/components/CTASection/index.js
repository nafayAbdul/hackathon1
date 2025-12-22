import React, { useState, useEffect, useRef } from 'react';

const CTASection = () => {
  const [isVisible, setIsVisible] = useState(false);
  const sectionRef = useRef(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          // Only trigger animation once
          observer.disconnect();
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

  return (
    <section ref={sectionRef} className="cta-section">
      <div className="container">
        <div className={`cta-content ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
          <h2 className={isVisible ? 'fade-in-up visible' : 'fade-in-up'}>Join the Physical AI Revolution</h2>
          <p className={isVisible ? 'fade-in-up visible' : 'fade-in-up'} style={{transitionDelay: '0.2s'}}>Be among the first to master the skills that will define the next decade of robotics and embodied intelligence. Limited early access available.</p>
          <div className={`cta-buttons ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`} style={{transitionDelay: '0.4s'}}>
            <a href="#" className="cta-button" style={{padding: '16px 40px', fontSize: '1.2rem'}}>
              <i className="fas fa-book-open mr-2"></i>Get Early Access
            </a>
            <a href="https://discord.com/invite/ros" className="secondary-button cta-button" style={{padding: '16px 35px', fontSize: '1.2rem'}}>
              <i className="fab fa-discord mr-2"></i>Join Community
            </a>
          </div>
        </div>
      </div>
    </section>
  );
};

export default CTASection;