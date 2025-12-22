# Research Summary: Physical AI & Humanoid Robotics Docusaurus UI Theme

## Overview
This research document captures all technical decisions, best practices, and implementation details required to adapt the Physical AI & Humanoid Robotics static HTML/CSS/JS theme into a Docusaurus-based documentation website.

## 1. Docusaurus Theme Customization

### Decision: Custom Theme Components
- **Rationale**: Docusaurus allows theme customization via src/theme/ directory to override default components
- **Implementation**: Create custom Navbar and Footer components that implement scroll-based styling and mobile overlay behavior

### Decision: Layout Wrapper
- **Rationale**: Specification requires using @theme/Layout wrapper as mandated by Docusaurus architecture
- **Implementation**: Homepage component will import and wrap content with @theme/Layout

## 2. CSS Variables and Global Styling

### Decision: CSS Custom Properties
- **Rationale**: Specification mandates exact color values as CSS variables in custom.css
- **Implementation**: Define the specified design tokens in src/css/custom.css:
  ```css
  :root {
    --primary: #6366f1;
    --secondary: #8b5cf6;
    --accent: #ec4899;
    --dark: #0f172a;
    --darker: #0a0f1d;
    --light: #f1f5f9;
    --gray: #94a3b8;
    --success: #10b981;
  }
  ```

### Decision: Typography Override
- **Rationale**: Specification requires Orbitron for logo/headings and Inter for body/buttons
- **Implementation**: Load Google Fonts and apply via CSS selectors in custom.css

## 3. Interactive Elements Implementation

### Decision: Particle Background with particles.js
- **Rationale**: Specification specifically mentions particles.js for background effects
- **Implementation**: Use react-tsparticles library to integrate with React/Docusaurus
- **Constraint**: Initialize only on client-side to avoid SSR issues

### Decision: Intersection Observer for Animations
- **Rationale**: Specification requires revealing elements when they enter viewport
- **Implementation**: Create custom hook using useEffect and IntersectionObserver API
- **Constraint**: Apply "visible" class once when element enters viewport

### Decision: Scroll-Based Navbar Effects
- **Rationale**: Specification requires navbar to transition when scrollY > 100px
- **Implementation**: Use scroll event listener with useEffect to detect scroll position
- **Apply scrolled class when threshold exceeded

## 4. Responsive Design Implementation

### Decision: Breakpoint-Specific Behavior
- **Rationale**: Specification defines exact responsive behavior at 992px and 768px
- **Implementation**: Use CSS media queries and React state to handle responsive changes
- **Specifics**: 
  - ≥992px: Show hero SVG, horizontal navbar
  - ≤992px: Hide hero SVG
  - ≤768px: Mobile navbar, reduced heading sizes, vertical navigation

## 5. Animation and Interaction Details

### Decision: CTA Button Effects
- **Rationale**: Specification details specific hover and idle animations
- **Implementation**: 
  - Hover: Vertical lift with shadow amplification and shimmer sweep
  - Idle: Repeating pulse shadow every 2 seconds
- **Approach**: CSS animations and transforms with :hover pseudo-class

### Decision: Section Animations
- **Rationale**: Features and Learning Objectives sections need specific animations
- **Implementation**:
  - Features: Fade-in upward on reveal, hover elevation and border glow
  - Learning Objectives: Slide-in from left on reveal, hover horizontal shift

## 6. Accessibility Implementation

### Decision: WCAG 2.1 AA Compliance
- **Rationale**: Specification requires WCAG 2.1 AA standards compliance
- **Implementation**: 
  - Proper contrast ratios (verified with CSS variables)
  - Keyboard navigation support
  - ARIA attributes where needed
  - Semantic HTML structure

## 7. Performance Optimization

### Decision: Performance Targets
- **Rationale**: Specification sets performance goals (3s load time, 60fps animations)
- **Implementation**:
  - Optimize particle background for performance
  - Use CSS transforms and opacity for animations
  - Implement proper cleanup for scroll and intersection observers
  - Font loading strategy with timeout fallback

## 8. Browser Support Strategy

### Decision: Modern Browser Support
- **Rationale**: Specification requires support for latest 2 versions of major browsers
- **Implementation**: Use feature detection and graceful degradation where needed
- **Fallbacks**: Ensure core content remains accessible when advanced features fail

## 9. Security Implementation

### Decision: Client-Side Security Practices
- **Rationale**: Specification requires security best practices
- **Implementation**:
  - Content Security Policy (CSP) in Docusaurus config
  - Proper handling of any user inputs (though minimal in static site)
  - XSS protection through proper templating

## 10. Error Handling and Fallbacks

### Decision: Graceful Degradation
- **Rationale**: Specification requires fallback behavior when features fail
- **Implementation**:
  - Static background when particle background fails
  - Fallback fonts with 3s timeout
  - CSS-only styling when JavaScript disabled
  - Visible elements without animations when Intersection Observer unavailable