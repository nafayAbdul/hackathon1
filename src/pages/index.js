import React from 'react';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';
import FeaturesSection from '@site/src/components/FeaturesSection';
import LearningObjectivesSection from '@site/src/components/LearningObjectives';
import TechStackSection from '@site/src/components/TechStack';
import CTASection from '@site/src/components/CTASection';
import MainPurposeSection from '@site/src/components/MainPurpose';

export default function Home() {
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="The definitive 2025 guide to building intelligent systems that perceive, reason, and act in the physical world">
      <HeroSection />
      <MainPurposeSection />
      <FeaturesSection />
      <TechStackSection />
      <CTASection />
      <LearningObjectivesSection />
    </Layout>
  );
}