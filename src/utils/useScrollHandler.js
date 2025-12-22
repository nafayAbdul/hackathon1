import { useState, useEffect } from 'react';

/**
 * Custom hook to handle scroll-based UI changes
 * @returns {{isScrolled: boolean, navbarRef: React.RefObject}}
 */
const useScrollHandler = () => {
  const [isScrolled, setIsScrolled] = useState(false);
  const navbarRef = { current: null };

  useEffect(() => {
    const handleScroll = () => {
      // Returns true when scrollY > 100px
      setIsScrolled(window.scrollY > 100);
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

  return { isScrolled, navbarRef };
};

export default useScrollHandler;