# Data Model: Physical AI & Humanoid Robotics Docusaurus UI Theme

## Overview
This document defines the key entities and components for the Physical AI & Humanoid Robotics Docusaurus UI Theme. Since this is primarily a UI/visual implementation with minimal data requirements, the "data model" focuses on component structures and their properties.

## 1. Design Tokens (CSS Variables)

### Entity: DesignTokens
- **Description**: Color palette and typography specifications that define the visual identity
- **Properties**:
  - `primary`: #6366f1 (violet/blue accent)
  - `secondary`: #8b5cf6 (purple accent)
  - `accent`: #ec4899 (pink accent)
  - `dark`: #0f172a (dark blue background)
  - `darker`: #0a0f1d (darker blue background)
  - `light`: #f1f5f9 (light gray text/background)
  - `gray`: #94a3b8 (medium gray text)
  - `success`: #10b981 (green for success states)

## 2. Layout Components

### Entity: NavigationBar
- **Description**: Fixed-position navigation with scroll-based styling
- **Properties**:
  - `position`: fixed (always at top)
  - `scrolledState`: boolean (applies when scrollY > 100px)
  - `backdropBlur`: boolean (applies when scrolled)
  - `mobileOverlay`: boolean (full-width overlay on mobile)
  - `links`: array of navigation links
- **Validation**: Must maintain 100px scroll threshold for scrolled state

### Entity: HeroSection
- **Description**: Full viewport height section with particle background
- **Properties**:
  - `minHeight`: 100vh (minimum height)
  - `contentAlignment`: left-aligned text content
  - `svgVisibility`: visible only on desktop (≥992px)
  - `particleBackground`: canvas-based particle system
  - `radialGradient`: rotating gradient overlay
  - `contentAnimation`: fade-in upward on mount
  - `svgAnimation`: continuous vertical float
- **Validation**: Must maintain minimum height of 100vh

### Entity: FeaturesSection
- **Description**: Grid layout for feature cards
- **Properties**:
  - `layout`: CSS Grid with auto-fit columns
  - `minWidth`: 300px per grid item
  - `animation`: initially hidden, revealed via IntersectionObserver
  - `hoverEffect`: elevation and border glow
- **Validation**: Must use auto-fit grid with minimum 300px width

### Entity: LearningObjectivesSection
- **Description**: Grid layout for learning objectives
- **Properties**:
  - `layout`: CSS Grid with auto-fit columns
  - `minWidth`: 250px per grid item
  - `animation`: slide-in from left on reveal
  - `accent`: left border accent
  - `hoverEffect`: horizontal shift
- **Validation**: Must slide in from left with left border accent

### Entity: Footer
- **Description**: Multi-column grid footer
- **Properties**:
  - `layout`: Multi-column grid structure
  - `brandText`: Brand information section
  - `navigationLinks`: Site navigation links
  - `communityLinks`: Community-related links
- **Validation**: Must implement multi-column grid structure

## 3. Animation Controllers

### Entity: ScrollHandler
- **Description**: Handles scroll-based UI changes
- **Properties**:
  - `scrollThreshold`: 100px (triggers navbar scrolled state)
  - `sectionObserver`: IntersectionObserver for revealing sections
  - `smoothScrolling`: For anchor link navigation
- **Validation**: Must apply scrolled class when scrollY > 100px

### Entity: ParticleSystem
- **Description**: Canvas-based particle background
- **Properties**:
  - `clientOnly`: Initializes only on client-side
  - `fillViewport`: Canvas fills entire viewport
  - `hoverGrab`: Particles react to mouse movement
  - `clickParticles`: Adds particles on click
- **Validation**: Must initialize only on client-side to avoid SSR errors

### Entity: IntersectionObserverController
- **Description**: Handles element reveal animations
- **Properties**:
  - `targetElements`: Feature cards and learning objectives
  - `triggerOnce`: Applies visible class only once
  - `visibleClass`: CSS class to apply when visible
- **Validation**: Must apply visible class once when element enters viewport

## 4. UI Components

### Entity: CTAButton
- **Description**: Call-to-action buttons with special effects
- **Properties**:
  - `hoverLift`: Vertical lift effect on hover
  - `shadowAmplification`: Enhanced shadow on hover
  - `shimmerSweep`: Shimmer effect on hover
  - `pulseShadow`: Repeating pulse shadow every 2 seconds when idle
- **Validation**: Must exhibit all specified hover and idle effects

### Entity: ResponsiveBreakpoints
- **Description**: Defines responsive behavior at different screen sizes
- **Properties**:
  - `desktopShowSvg`: ≥992px Hero SVG visible
  - `desktopNavbar`: ≥992px Horizontal navbar
  - `tabletHideSvg`: ≤992px Hero SVG hidden
  - `mobileNavbar`: ≤768px Mobile navbar style
  - `mobileHeadingSizes`: ≤768px Reduced heading sizes
  - `mobileVerticalNav`: ≤768px Vertical navigation layout
- **Validation**: Must follow all specified breakpoint behaviors