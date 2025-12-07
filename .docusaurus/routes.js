import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-book/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug', '12f'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/config', '4d3'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/content', 'a5b'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/globalData', 'abe'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/metadata', '587'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/registry', '2ef'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/routes', '1a5'),
    exact: true
  },
  {
    path: '/physical-ai-book/docs',
    component: ComponentCreator('/physical-ai-book/docs', '465'),
    routes: [
      {
        path: '/physical-ai-book/docs',
        component: ComponentCreator('/physical-ai-book/docs', 'a30'),
        routes: [
          {
            path: '/physical-ai-book/docs',
            component: ComponentCreator('/physical-ai-book/docs', '2b7'),
            routes: [
              {
                path: '/physical-ai-book/docs/chapter1_digital_to_embodied',
                component: ComponentCreator('/physical-ai-book/docs/chapter1_digital_to_embodied', '430'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter1_exercises',
                component: ComponentCreator('/physical-ai-book/docs/chapter1_exercises', '3be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter2_exercises',
                component: ComponentCreator('/physical-ai-book/docs/chapter2_exercises', '1cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter2_ros2_fundamentals',
                component: ComponentCreator('/physical-ai-book/docs/chapter2_ros2_fundamentals', '7a1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter3_exercises',
                component: ComponentCreator('/physical-ai-book/docs/chapter3_exercises', '861'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter3_rclpy_ai_agents',
                component: ComponentCreator('/physical-ai-book/docs/chapter3_rclpy_ai_agents', 'c4f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter4_exercises',
                component: ComponentCreator('/physical-ai-book/docs/chapter4_exercises', 'daa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter4_urdf_xacro_mastery',
                component: ComponentCreator('/physical-ai-book/docs/chapter4_urdf_xacro_mastery', '0cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter5_complete_ros2_package',
                component: ComponentCreator('/physical-ai-book/docs/chapter5_complete_ros2_package', '6d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/chapter5_exercises',
                component: ComponentCreator('/physical-ai-book/docs/chapter5_exercises', 'e1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/intro',
                component: ComponentCreator('/physical-ai-book/docs/intro', 'f2d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module1_intro',
                component: ComponentCreator('/physical-ai-book/docs/module1_intro', '860'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module1/chapter1_digital_to_embodied',
                component: ComponentCreator('/physical-ai-book/docs/module1/chapter1_digital_to_embodied', '2b0'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module1/chapter2_ros2_fundamentals',
                component: ComponentCreator('/physical-ai-book/docs/module1/chapter2_ros2_fundamentals', 'f09'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module1/chapter3_rclpy_ai_agents',
                component: ComponentCreator('/physical-ai-book/docs/module1/chapter3_rclpy_ai_agents', '0c5'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module1/chapter4_urdf_xacro_mastery',
                component: ComponentCreator('/physical-ai-book/docs/module1/chapter4_urdf_xacro_mastery', 'a17'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module1/chapter5_complete_ros2_package',
                component: ComponentCreator('/physical-ai-book/docs/module1/chapter5_complete_ros2_package', '032'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module1/intro',
                component: ComponentCreator('/physical-ai-book/docs/module1/intro', '9f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical-ai-book/',
    component: ComponentCreator('/physical-ai-book/', '003'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
