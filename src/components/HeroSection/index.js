import React, { useState, useEffect, useRef } from 'react';
import ParticleBackground from '../ParticleBackground';

const HeroSection = () => {
  // Define the missing isVisible state
  const [isVisible, setIsVisible] = useState(false);
  const projectsCountRef = useRef(null);
  const codeCountRef = useRef(null);
  const interviewsCountRef = useRef(null);

  // Counter animation function
  const animateCounter = (elementRef, target, duration = 2000, addPlus = false) => {
    if (!elementRef.current) return;

    let start = 0;
    const increment = target / (duration / 16); // 16ms ~ 60fps

    const timer = setInterval(() => {
      start += increment;
      if (start >= target) {
        elementRef.current.textContent = target + (addPlus ? '+' : '');
        clearInterval(timer);
      } else {
        elementRef.current.textContent = Math.floor(start) + (addPlus ? '+' : '');
      }
    }, 16);
  };

  useEffect(() => {
    // Trigger the animation shortly after mount
    const timer = setTimeout(() => setIsVisible(true), 100);
    return () => clearTimeout(timer);
  }, []);

  // Start counters when component is visible
  useEffect(() => {
    // Delay counter animation to start after fade-in
    const counterTimer = setTimeout(() => {
      animateCounter(projectsCountRef, 45, 2000, true); // Add '+' to projects count
      animateCounter(codeCountRef, 100, 2000, false);
      animateCounter(interviewsCountRef, 20, 2000, false);
    }, 800); // Wait for fade-in to complete

    return () => clearTimeout(counterTimer);
  }, []); // Empty dependency array to run only once after initial render

  return (
    <section className="hero-section">
      {/* Animated background elements */}
      <div className="absolute inset-0 overflow-hidden">
        {/* Floating circuit patterns */}
        <div className="floating-circle-1"></div>
        <div className="floating-circle-2"></div>
        <div className="floating-circle-3"></div>

        {/* Animated grid lines */}
        <div className="grid-lines"></div>

        {/* Floating robotic icons */}
        {[...Array(8)].map((_, i) => (
          <div key={i} className="floating-robotic-element" style={{
            left: `${10 + i * 12}%`,
            top: `${20 + (i % 3) * 25}%`,
          }}></div>
        ))}
      </div>

      {/* Animated bottom accent */}
      <div className="bottom-accent-bar"></div>

      <ParticleBackground />
      <div className="hero-content-container">
        <div className={`hero-content ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
          <div className="enrollment-badge">
            <div className="pulse-dot"></div>
            <span className="enrollment-text">Enrollment Open</span>
          </div>

          <h1 className={`hero-title ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
            Mastering Physical AI & <br />
            Humanoid Robotics
          </h1>
          <p className={`hero-subtitle ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
            The definitive 2025 guide to building intelligent systems that perceive, reason, and act in the physical world. From simulation to real-world deployment.
          </p>

          <div className={`hero-stats ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
            <div className="stat-item">
              <div className="stat-value" ref={projectsCountRef}>0</div>
              <div className="stat-label">Hands-on Projects</div>
            </div>
            <div className="stat-item">
              <div className="stat-value" ref={codeCountRef}>0</div>
              <div className="stat-label">Code Examples</div>
            </div>
            <div className="stat-item">
              <div className="stat-value" ref={interviewsCountRef}>0</div>
              <div className="stat-label">Expert Interviews</div>
            </div>
          </div>

          <div className={`hero-cta-container ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
            <a href="/docs/module1/intro" className="cta-button">
              Start Reading <i className="fas fa-arrow-right ml-2"></i>
            </a>
            <a href="/about" className="cta-button cta-button--secondary">
              About Us
            </a>
          </div>
        </div>
        <div className={`hero-svg-container ${isVisible ? 'fade-in-up visible' : 'fade-in-up'}`}>
          {/* Decorative SVG - floating animation applied via CSS */}
          <div className="robot-container">
            <div className="floating-robot">
              <svg viewBox="0 0 500 600" xmlns="http://www.w3.org/2000/svg">
                {/* Robot body with gradient */}
                <defs>
                  <linearGradient id="robotGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                    <stop offset="0%" stopColor="#0ea5e9" />
                    <stop offset="100%" stopColor="#14b8a6" />
                  </linearGradient>
                  <linearGradient id="eyeGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                    <stop offset="0%" stopColor="#fbbf24" />
                    <stop offset="100%" stopColor="#f59e0b" />
                  </linearGradient>
                  <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
                    <feGaussianBlur stdDeviation="3" result="blur" />
                    <feComposite in="SourceGraphic" in2="blur" operator="over" />
                  </filter>
                </defs>

                {/* Head */}
                <ellipse cx="250" cy="180" rx="80" ry="70" fill="url(#robotGradient)" stroke="#0c4a6e" strokeWidth="4" filter="url(#glow)" />

                {/* Eyes */}
                <circle cx="220" cy="170" r="15" fill="url(#eyeGradient)" stroke="#b45309" strokeWidth="2" filter="url(#glow)" />
                <circle cx="280" cy="170" r="15" fill="url(#eyeGradient)" stroke="#b45309" strokeWidth="2" filter="url(#glow)" />

                {/* Eye highlights */}
                <circle cx="225" cy="165" r="5" fill="white" />
                <circle cx="285" cy="165" r="5" fill="white" />

                {/* Body */}
                <rect x="170" y="250" width="160" height="200" rx="30" fill="url(#robotGradient)" stroke="#0c4a6e" strokeWidth="4" filter="url(#glow)" />

                {/* Arms with animation */}
                <rect x="80" y="280" width="50" height="160" rx="15" fill="url(#robotGradient)" stroke="#0c4a6e" strokeWidth="3" />
                <rect x="370" y="280" width="50" height="160" rx="15" fill="url(#robotGradient)" stroke="#0c4a6e" strokeWidth="3" />

                {/* Hands */}
                <circle cx="105" cy="440" r="25" fill="#1e293b" stroke="#0c4a6e" strokeWidth="2" />
                <circle cx="395" cy="440" r="25" fill="#1e293b" stroke="#0c4a6e" strokeWidth="2" />

                {/* Legs */}
                <rect x="200" y="450" width="40" height="100" rx="10" fill="url(#robotGradient)" stroke="#0c4a6e" strokeWidth="3" />
                <rect x="260" y="450" width="40" height="100" rx="10" fill="url(#robotGradient)" stroke="#0c4a6e" strokeWidth="3" />

                {/* Feet */}
                <rect x="190" y="550" width="60" height="20" rx="10" fill="#1e293b" stroke="#0c4a6e" strokeWidth="2" />
                <rect x="250" y="550" width="60" height="20" rx="10" fill="#1e293b" stroke="#0c4a6e" strokeWidth="2" />

                {/* Antenna with pulsing animation */}
                <line x1="250" y1="110" x2="250" y2="80" stroke="#0c4a6e" strokeWidth="4" strokeLinecap="round" />
                <circle cx="250" cy="70" r="10" fill="#f43f5e" stroke="#b91c1c" strokeWidth="2" filter="url(#glow)" id="processorLight" />

                {/* Circuit patterns */}
                <path d="M200 300 Q225 320 250 300 T300 300" fill="none" stroke="#0c4a6e" strokeWidth="2" strokeDasharray="5,5" />
                <path d="M200 350 Q225 370 250 350 T300 350" fill="none" stroke="#0c4a6e" strokeWidth="2" strokeDasharray="5,5" />
                <path d="M200 400 Q225 420 250 400 T300 400" fill="none" stroke="#0c4a6e" strokeWidth="2" strokeDasharray="5,5" />

                {/* Processor light - blinking animation */}
                <circle cx="250" cy="350" r="15" fill="#22c55e" stroke="#15803d" strokeWidth="2" filter="url(#glow)" />

                {/* Display panel */}
                <rect x="200" y="300" width="100" height="50" rx="8" fill="#0f172a" stroke="#334155" strokeWidth="2" />
                <text x="250" y="330" fontFamily="Arial" fontSize="12" fill="#0ea5e9" textAnchor="middle">AI READY</text>
              </svg>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;