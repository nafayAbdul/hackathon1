import React, { useCallback } from 'react';
import { Particles } from '@tsparticles/react';
import { loadSlim } from '@tsparticles/slim'; // Using slim version instead of full

const ParticleBackground = () => {
  const initParticles = useCallback(async (engine) => {
    // Initialize particles.js engine
    await loadSlim(engine);
  }, []);

  // Configuration for the particle background
  const particlesOptions = {
    background: {
      color: {
        value: 'transparent',
      },
    },
    fpsLimit: 120,
    interactivity: {
      events: {
        onClick: {
          enable: true,
          mode: 'push',
        },
        onHover: {
          enable: true,
          mode: 'grab',
        },
        resize: true,
      },
      modes: {
        grab: {
          distance: 150,
          links: {
            opacity: 0.8,
          },
        },
        push: {
          quantity: 4,
        },
      },
    },
    particles: {
      color: {
        value: '#6366f1',
      },
      links: {
        color: '#8b5cf6',
        distance: 150,
        enable: true,
        opacity: 0.5,
        width: 1,
      },
      move: {
        direction: 'none',
        enable: true,
        outModes: {
          default: 'bounce',
        },
        random: false,
        speed: 2,
        straight: false,
      },
      number: {
        density: {
          enable: true,
          area: 800,
        },
        value: 40,
      },
      opacity: {
        value: 0.5,
      },
      shape: {
        type: 'circle',
      },
      size: {
        value: { min: 1, max: 5 },
      },
    },
    detectRetina: true,
  };

  return (
    <div className="particle-background">
      <Particles
        id="tsparticles"
        init={initParticles}
        options={particlesOptions}
      />
    </div>
  );
};

export default ParticleBackground;