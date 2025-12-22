# Quickstart Guide: Physical AI & Humanoid Robotics Docusaurus UI Theme

## Overview
This guide provides the essential steps to set up, develop, and deploy the Physical AI & Humanoid Robotics Docusaurus UI Theme.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

## Setup Instructions

### 1. Clone and Initialize
```bash
# If starting from scratch
npx create-docusaurus@latest website classic

# Or if working with existing repo
git clone <repository-url>
cd <repository-directory>
npm install
```

### 2. Install Required Dependencies
```bash
npm install --save react-tsparticles tsparticles
npm install --save @fontsource/orbitron @fontsource/inter
```

### 3. Project Structure
The theme implementation follows this structure:
```
src/
├── components/          # Custom React components
│   ├── HeroSection/
│   ├── FeaturesSection/
│   ├── LearningObjectives/
│   └── ParticleBackground/
├── pages/
│   └── index.js         # Homepage with @theme/Layout wrapper
├── theme/
│   ├── Navbar/          # Custom Navbar override
│   └── Footer/          # Custom Footer override
├── css/
│   └── custom.css       # Global styles and CSS variables
└── utils/               # Utility functions (e.g., scroll handlers)
```

## Development Workflow

### 1. Start Development Server
```bash
npm run start
```
This starts the development server at http://localhost:3000

### 2. Key Implementation Areas

#### Custom CSS Variables
Update `src/css/custom.css` with the required design tokens:
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

#### Homepage Implementation
Edit `src/pages/index.js` to implement the homepage structure with:
- Hero section with particle background
- Features section with grid layout
- Learning Objectives section
- All using @theme/Layout wrapper

#### Navbar Override
Create custom Navbar component in `src/theme/Navbar` to implement:
- Fixed positioning
- Scroll-based styling (scrolled class at >100px)
- Mobile overlay behavior

#### Footer Override
Create custom Footer component in `src/theme/Footer` with multi-column grid structure.

## Key Features Implementation

### 1. Particle Background
- Initialize only on client-side to prevent SSR errors
- Use react-tsparticles for the particle effect
- Implement hover grab and click to add particles

### 2. Intersection Observer Animations
- Create a custom hook using useEffect and IntersectionObserver API
- Apply "visible" class when elements enter viewport
- Trigger once per element

### 3. Responsive Design
- Implement breakpoints at ≥992px and ≤768px
- Hide/show hero SVG based on screen size
- Adjust navigation layout for mobile

### 4. CTA Button Effects
- Hover: vertical lift, shadow amplification, shimmer sweep
- Idle: repeating pulse shadow every 2 seconds

## Testing and Validation

### 1. Manual Testing Checklist
- [ ] Homepage renders without React/console errors
- [ ] Particle background initializes only on client
- [ ] Navbar applies scrolled class at >100px scroll
- [ ] Mobile navigation toggles correctly
- [ ] Feature cards reveal with animation when scrolled into view
- [ ] Learning objectives slide in from left
- [ ] CTA buttons exhibit hover and idle effects
- [ ] Responsive behavior works at all breakpoints
- [ ] All content accessible when JavaScript is disabled

### 2. Performance Testing
- [ ] Page loads under 3 seconds on 3G connection
- [ ] Animations maintain 60fps on mid-range devices
- [ ] Particle background performs well without lag

## Build and Deployment

### 1. Build for Production
```bash
npm run build
```

### 2. Preview Production Build
```bash
npm run serve
```

### 3. Deploy
The build output in the `build/` directory can be deployed to any static hosting service, or specifically to:
- Vercel (recommended for Docusaurus)
- GitHub Pages
- Netlify
- Any static hosting platform