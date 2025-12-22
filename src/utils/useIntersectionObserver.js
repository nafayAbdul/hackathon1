import { useEffect, useRef } from 'react';

/**
 * Custom hook to observe elements and trigger visibility changes
 * @param {Object} options - IntersectionObserver options
 * @returns {{observe: Function, unobserve: Function}}
 */
const useIntersectionObserver = (options = {}) => {
  const observerRef = useRef(null);
  const elementsRef = useRef(new Map());

  useEffect(() => {
    // Default options for the IntersectionObserver
    const defaultOptions = {
      root: null,
      rootMargin: '0px',
      threshold: 0.1, // Trigger when 10% of the element is visible
      ...options
    };

    // Create the IntersectionObserver instance
    observerRef.current = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (entry.isIntersecting) {
          // Apply 'visible' class once when element enters viewport
          entry.target.classList.add('visible');
          
          // Stop observing this element after it becomes visible once
          if (observerRef.current) {
            observerRef.current.unobserve(entry.target);
          }
          
          // Remove from our tracking map
          elementsRef.current.delete(entry.target);
        }
      });
    }, defaultOptions);

    // Clean up on unmount
    return () => {
      if (observerRef.current) {
        observerRef.current.disconnect();
      }
      elementsRef.current.clear();
    };
  }, [options]);

  const observe = (element) => {
    if (observerRef.current && element) {
      observerRef.current.observe(element);
      elementsRef.current.set(element, true);
    }
  };

  const unobserve = (element) => {
    if (observerRef.current && element) {
      observerRef.current.unobserve(element);
      elementsRef.current.delete(element);
    }
  };

  return { observe, unobserve };
};

export default useIntersectionObserver;