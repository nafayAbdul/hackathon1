# Component Interface Contracts: Physical AI & Humanoid Robotics Docusaurus UI Theme

## Overview
This document specifies the interface contracts for the custom components in the Physical AI & Humanoid Robotics Docusaurus UI Theme. Since this is primarily a frontend project, the contracts focus on component APIs and data interfaces.

## 1. Homepage Component

### Component: `src/pages/index.js`
- **Purpose**: Main homepage with @theme/Layout wrapper
- **Props**: None (self-contained)
- **Children**: 
  - HeroSection
  - FeaturesSection
  - LearningObjectivesSection
  - Footer (via Layout)
- **Dependencies**: @theme/Layout, custom CSS, Google Fonts
- **State**: None (functional component)
- **Effects**: 
  - Particle initialization (client-side only)
  - Scroll event listener setup/teardown

## 2. Navigation Components

### Component: `src/theme/Navbar/index.js`
- **Purpose**: Custom Navbar with scroll-based styling
- **Props**: 
  - `logo`: Logo configuration object
  - `title`: Site title
  - `items`: Navigation items array
- **State**:
  - `isScrolled`: Boolean indicating scroll state (>100px)
  - `isMobileOpen`: Boolean for mobile menu state
- **Effects**:
  - Scroll event listener to detect scroll position
  - Apply/remove 'scrolled' class based on scroll position

### Component: `src/theme/Footer/index.js`
- **Purpose**: Custom multi-column footer
- **Props**:
  - `links`: Array of navigation link objects
  - `copyright`: Copyright text
  - `social`: Social media links
- **State**: None (functional component)

## 3. Section Components

### Component: `src/components/HeroSection/index.js`
- **Purpose**: Hero section with particle background
- **Props**: None (self-contained)
- **State**:
  - `contentVisible`: Boolean for fade-in animation
- **Effects**:
  - Initialize particle background (client-side only)
  - Apply content animation on mount
- **Children**: ParticleBackground, content elements

### Component: `src/components/FeaturesSection/index.js`
- **Purpose**: Grid layout for feature cards
- **Props**: 
  - `features`: Array of feature objects
- **State**: None (uses IntersectionObserver hook)
- **Effects**:
  - Set up IntersectionObserver for each card
  - Apply 'visible' class when cards enter viewport
- **Feature Object Structure**:
  - `title`: String (feature title)
  - `description`: String (feature description)
  - `icon`: String or React element (feature icon)

### Component: `src/components/LearningObjectivesSection/index.js`
- **Purpose**: Grid layout for learning objectives
- **Props**: 
  - `objectives`: Array of objective objects
- **State**: None (uses IntersectionObserver hook)
- **Effects**:
  - Set up IntersectionObserver for each objective
  - Apply 'visible' class when objectives enter viewport
- **Objective Object Structure**:
  - `title`: String (objective title)
  - `description`: String (objective description)

## 4. Interactive Components

### Component: `src/components/ParticleBackground/index.js`
- **Purpose**: Canvas-based particle background
- **Props**: None (self-contained)
- **State**: None (uses refs for canvas manipulation)
- **Effects**:
  - Initialize particles.js on client-side only
  - Set up canvas and particle system
  - Handle mouse interactions (grab, click to add particles)
- **Constraints**: 
  - Must initialize only after component mounts (client-side)
  - Must clean up properly on unmount

### Component: `src/components/CTAButton/index.js`
- **Purpose**: Call-to-action button with special effects
- **Props**:
  - `children`: Button content
  - `href`: Link destination (optional)
  - `onClick`: Click handler (optional)
  - `variant`: Button style variant (optional)
- **State**: None (uses CSS for animations)
- **Effects**:
  - Set up pulse shadow animation (every 2 seconds when idle)

## 5. Utility Functions

### Hook: `src/utils/useIntersectionObserver.js`
- **Purpose**: Hook to observe elements and trigger visibility changes
- **Params**:
  - `options`: IntersectionObserver options object
- **Returns**: 
  - `observe`: Function to start observing an element
  - `unobserve`: Function to stop observing an element
- **Behavior**: 
  - Applies 'visible' class once when element enters viewport
  - Only triggers once per element

### Hook: `src/utils/useScrollHandler.js`
- **Purpose**: Hook to handle scroll-based UI changes
- **Params**: None
- **Returns**:
  - `isScrolled`: Boolean indicating scroll state
  - `navbarRef`: Ref to attach to navbar
- **Behavior**:
  - Returns true when scrollY > 100px
  - Sets up and cleans up scroll event listener

## 6. CSS Classes Contract

### CSS Classes for Animations
- `visible`: Applied by IntersectionObserver when elements enter viewport
- `scrolled`: Applied to navbar when scrollY > 100px
- `fade-in-up`: Animation for content in hero section
- `float`: Animation for SVG in hero section
- `elevate`: Hover effect for feature cards
- `border-glow`: Hover effect for feature cards
- `slide-in-left`: Animation for learning objectives
- `horizontal-shift`: Hover effect for learning objectives
- `pulse-shadow`: Idle animation for CTA buttons
- `lift`: Hover effect for CTA buttons
- `shimmer-sweep`: Hover effect for CTA buttons

### CSS Variables (from custom.css)
- `--primary`: #6366f1
- `--secondary`: #8b5cf6
- `--accent`: #ec4899
- `--dark`: #0f172a
- `--darker`: #0a0f1d
- `--light`: #f1f5f9
- `--gray`: #94a3b8
- `--success`: #10b981