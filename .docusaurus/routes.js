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
    component: ComponentCreator('/physical-ai-book/docs', '470'),
    routes: [
      {
        path: '/physical-ai-book/docs',
        component: ComponentCreator('/physical-ai-book/docs', '7cf'),
        routes: [
          {
            path: '/physical-ai-book/docs',
            component: ComponentCreator('/physical-ai-book/docs', '91f'),
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
              },
              {
                path: '/physical-ai-book/docs/module2/chapter10_closing_sim_loop',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter10_closing_sim_loop', '36d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter10_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter10_exercises', '3f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter6_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter6_exercises', 'b44'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter6_simulation_2025',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter6_simulation_2025', '764'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter7_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter7_exercises', 'dcc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter7_realistic_sensors',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter7_realistic_sensors', '63a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter8_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter8_exercises', '6f4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter8_photorealistic_rendering',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter8_photorealistic_rendering', 'ffe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter9_domain_randomization',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter9_domain_randomization', '2a4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/chapter9_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module2/chapter9_exercises', '70e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module2/intro',
                component: ComponentCreator('/physical-ai-book/docs/module2/intro', 'c73'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/',
                component: ComponentCreator('/physical-ai-book/docs/module3/', '4fe'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module3/chapter11_simulation_2025',
                component: ComponentCreator('/physical-ai-book/docs/module3/chapter11_simulation_2025', '557'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/chapter12_ros2_fundamentals',
                component: ComponentCreator('/physical-ai-book/docs/module3/chapter12_ros2_fundamentals', 'bd1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/chapter13_advanced_navigation',
                component: ComponentCreator('/physical-ai-book/docs/module3/chapter13_advanced_navigation', 'bf4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/chapter14_reinforcement_learning',
                component: ComponentCreator('/physical-ai-book/docs/module3/chapter14_reinforcement_learning', 'fc4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/chapter15_sim_to_real_transfer',
                component: ComponentCreator('/physical-ai-book/docs/module3/chapter15_sim_to_real_transfer', '830'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/intro',
                component: ComponentCreator('/physical-ai-book/docs/module3/intro', '034'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module3/summary',
                component: ComponentCreator('/physical-ai-book/docs/module3/summary', '937'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/',
                component: ComponentCreator('/physical-ai-book/docs/module4/', 'a91'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module4/chapter16_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter16_exercises', 'b47'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter16_vla_revolution',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter16_vla_revolution', 'fbf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter17_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter17_exercises', 'ea7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter17_fine_tuning',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter17_fine_tuning', 'b9b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter17_vla_finetuning',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter17_vla_finetuning', 'd89'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module4/chapter18_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter18_exercises', 'e2c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter18_voice_action_pipeline',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter18_voice_action_pipeline', 'ee4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter19_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter19_exercises', '7b8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter19_multi_modal_foundations',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter19_multi_modal_foundations', '9b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter20_exercises',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter20_exercises', 'e76'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/chapter20_sim_to_real_transfer',
                component: ComponentCreator('/physical-ai-book/docs/module4/chapter20_sim_to_real_transfer', '9a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/intro',
                component: ComponentCreator('/physical-ai-book/docs/module4/intro', '139'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module4/quickstart',
                component: ComponentCreator('/physical-ai-book/docs/module4/quickstart', 'cb4'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module4/summary',
                component: ComponentCreator('/physical-ai-book/docs/module4/summary', '2ed'),
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
