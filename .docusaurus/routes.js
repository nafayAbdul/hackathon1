import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'e16'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f58'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'f51'),
            routes: [
              {
                path: '/docs/chapter1_digital_to_embodied',
                component: ComponentCreator('/docs/chapter1_digital_to_embodied', 'a6f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter1_exercises',
                component: ComponentCreator('/docs/chapter1_exercises', 'b82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter2_exercises',
                component: ComponentCreator('/docs/chapter2_exercises', 'ceb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter2_ros2_fundamentals',
                component: ComponentCreator('/docs/chapter2_ros2_fundamentals', '6af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter3_exercises',
                component: ComponentCreator('/docs/chapter3_exercises', '12e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter3_rclpy_ai_agents',
                component: ComponentCreator('/docs/chapter3_rclpy_ai_agents', '3c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter4_exercises',
                component: ComponentCreator('/docs/chapter4_exercises', 'e30'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter4_urdf_xacro_mastery',
                component: ComponentCreator('/docs/chapter4_urdf_xacro_mastery', 'e33'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter5_complete_ros2_package',
                component: ComponentCreator('/docs/chapter5_complete_ros2_package', '3ea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter5_exercises',
                component: ComponentCreator('/docs/chapter5_exercises', '825'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1_intro',
                component: ComponentCreator('/docs/module1_intro', 'cef'),
                exact: true
              },
              {
                path: '/docs/module1/chapter1_digital_to_embodied',
                component: ComponentCreator('/docs/module1/chapter1_digital_to_embodied', 'dbe'),
                exact: true
              },
              {
                path: '/docs/module1/chapter2_ros2_fundamentals',
                component: ComponentCreator('/docs/module1/chapter2_ros2_fundamentals', '9fc'),
                exact: true
              },
              {
                path: '/docs/module1/chapter3_rclpy_ai_agents',
                component: ComponentCreator('/docs/module1/chapter3_rclpy_ai_agents', '0f2'),
                exact: true
              },
              {
                path: '/docs/module1/chapter4_urdf_xacro_mastery',
                component: ComponentCreator('/docs/module1/chapter4_urdf_xacro_mastery', '490'),
                exact: true
              },
              {
                path: '/docs/module1/chapter5_complete_ros2_package',
                component: ComponentCreator('/docs/module1/chapter5_complete_ros2_package', 'aca'),
                exact: true
              },
              {
                path: '/docs/module1/intro',
                component: ComponentCreator('/docs/module1/intro', 'f8b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter10_closing_sim_loop',
                component: ComponentCreator('/docs/module2/chapter10_closing_sim_loop', 'ebe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter10_exercises',
                component: ComponentCreator('/docs/module2/chapter10_exercises', '8f8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter6_exercises',
                component: ComponentCreator('/docs/module2/chapter6_exercises', 'a57'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter6_simulation_2025',
                component: ComponentCreator('/docs/module2/chapter6_simulation_2025', 'b00'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter7_exercises',
                component: ComponentCreator('/docs/module2/chapter7_exercises', '433'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter7_realistic_sensors',
                component: ComponentCreator('/docs/module2/chapter7_realistic_sensors', '51c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter8_exercises',
                component: ComponentCreator('/docs/module2/chapter8_exercises', '5b1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter8_photorealistic_rendering',
                component: ComponentCreator('/docs/module2/chapter8_photorealistic_rendering', '860'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter9_domain_randomization',
                component: ComponentCreator('/docs/module2/chapter9_domain_randomization', '471'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter9_exercises',
                component: ComponentCreator('/docs/module2/chapter9_exercises', 'f87'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/intro',
                component: ComponentCreator('/docs/module2/intro', '1bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/',
                component: ComponentCreator('/docs/module3/', '578'),
                exact: true
              },
              {
                path: '/docs/module3/chapter11_simulation_2025',
                component: ComponentCreator('/docs/module3/chapter11_simulation_2025', '135'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter12_ros2_fundamentals',
                component: ComponentCreator('/docs/module3/chapter12_ros2_fundamentals', '1a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter13_advanced_navigation',
                component: ComponentCreator('/docs/module3/chapter13_advanced_navigation', 'ca9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter14_reinforcement_learning',
                component: ComponentCreator('/docs/module3/chapter14_reinforcement_learning', 'ef5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter15_sim_to_real_transfer',
                component: ComponentCreator('/docs/module3/chapter15_sim_to_real_transfer', '926'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/intro',
                component: ComponentCreator('/docs/module3/intro', '937'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/summary',
                component: ComponentCreator('/docs/module3/summary', '335'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/',
                component: ComponentCreator('/docs/module4/', '6f0'),
                exact: true
              },
              {
                path: '/docs/module4/chapter16_exercises',
                component: ComponentCreator('/docs/module4/chapter16_exercises', 'c56'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter16_vla_revolution',
                component: ComponentCreator('/docs/module4/chapter16_vla_revolution', '7db'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter17_exercises',
                component: ComponentCreator('/docs/module4/chapter17_exercises', 'fff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter17_fine_tuning',
                component: ComponentCreator('/docs/module4/chapter17_fine_tuning', 'a92'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter17_vla_finetuning',
                component: ComponentCreator('/docs/module4/chapter17_vla_finetuning', '83f'),
                exact: true
              },
              {
                path: '/docs/module4/chapter18_exercises',
                component: ComponentCreator('/docs/module4/chapter18_exercises', '3d4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter18_voice_action_pipeline',
                component: ComponentCreator('/docs/module4/chapter18_voice_action_pipeline', '9c2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter19_exercises',
                component: ComponentCreator('/docs/module4/chapter19_exercises', '23c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter19_multi_modal_foundations',
                component: ComponentCreator('/docs/module4/chapter19_multi_modal_foundations', 'a53'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter20_exercises',
                component: ComponentCreator('/docs/module4/chapter20_exercises', '7f7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter20_sim_to_real_transfer',
                component: ComponentCreator('/docs/module4/chapter20_sim_to_real_transfer', '271'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/intro',
                component: ComponentCreator('/docs/module4/intro', 'b26'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/quickstart',
                component: ComponentCreator('/docs/module4/quickstart', '001'),
                exact: true
              },
              {
                path: '/docs/module4/summary',
                component: ComponentCreator('/docs/module4/summary', '668'),
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
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
