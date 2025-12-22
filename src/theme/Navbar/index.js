import React, { useState, useEffect } from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import './Navbar.css';

const Navbar = (props) => {
  const [scrolled, setScrolled] = useState(false);
  const [mobileOpen, setMobileOpen] = useState(false);

  const toggleMobileMenu = () => {
    setMobileOpen(!mobileOpen);
  };

  // Handle scroll position and progress bar
  useEffect(() => {
    const handleScroll = () => {
      setScrolled(window.scrollY > 100); // Apply scrolled class when scrollY > 100px

      // Update scroll progress bar
      const scrollProgress = document.getElementById('scrollProgress');
      if (scrollProgress) {
        const scrollTop = document.documentElement.scrollTop;
        const scrollHeight = document.documentElement.scrollHeight - document.documentElement.clientHeight;
        const scrollPercent = (scrollTop / scrollHeight) * 100;
        scrollProgress.style.width = `${scrollPercent}%`;
      }
    };

    // Set initial state
    handleScroll();

    // Add scroll event listener
    window.addEventListener('scroll', handleScroll);

    // Clean up scroll event listener on unmount
    return () => {
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);

  // Apply scrolled class to the navbar inner element
  useEffect(() => {
    const navbarInner = document.querySelector('.navbar__inner');
    if (navbarInner) {
      if (scrolled) {
        navbarInner.classList.add('scrolled');
      } else {
        navbarInner.classList.remove('scrolled');
      }
    }
  }, [scrolled]);

  return (
    <>
      {/* Scroll Progress Bar */}
      <div className="scroll-progress" id="scrollProgress"></div>

      <div className={`navbar ${scrolled ? 'scrolled' : ''} ${mobileOpen ? 'mobile-open' : ''}`}>
        <OriginalNavbar {...props} />
      </div>
      {/* Mobile menu overlay */}
      {mobileOpen && (
        <div className="mobile-menu-overlay" onClick={toggleMobileMenu}>
          <div className="mobile-menu-content" onClick={(e) => e.stopPropagation()}>
            <button className="mobile-close-btn" onClick={toggleMobileMenu}>×</button>
            <ul>
              <li><a href="/">Home</a></li>
              <li><a href="#features">Features</a></li>
              <li><a href="#technology">Technology</a></li>
              <li><a href="#objectives">Objectives</a></li>
              <li><a href="https://github.com/nafayAbdul/hackathon1">GitHub</a></li>
            </ul>
          </div>
        </div>
      )}
    </>
  );
};

export default Navbar;