/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs




 - render a sidebar for each doc of that group
 - provide next/previous navigation



 /s


 npm npm///
 The sidebars can be generated from the filesystem, or explicitly defined here.

    ///
    
 Create as many sidebars as you want.
 */
module.exports = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'module1/intro',
        'chapter1_digital_to_embodied',
        {
          type: 'category',
          label: 'Chapter 1 Exercises',
          items: [
            'chapter1_exercises',
          ],
        },
        'chapter2_ros2_fundamentals',
        {
          type: 'category',
          label: 'Chapter 2 Exercises',
          items: [
            'chapter2_exercises',
          ],
        },
        'chapter3_rclpy_ai_agents',
        {
          type: 'category',
          label: 'Chapter 3 Exercises',
          items: [
            'chapter3_exercises',
          ],
        },
        'chapter4_urdf_xacro_mastery',
        {
          type: 'category',
          label: 'Chapter 4 Exercises',
          items: [
            'chapter4_exercises',
          ],
        },
        'chapter5_complete_ros2_package',
        {
          type: 'category',
          label: 'Chapter 5 Exercises',
          items: [
            'chapter5_exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Integration – The Digital Twin',
      items: [
        'module2/intro',
        'module2/chapter6_simulation_2025',
        {
          type: 'category',
          label: 'Chapter 6 Exercises',
          items: [
            'module2/chapter6_exercises',
          ],
        },
        'module2/chapter7_realistic_sensors',
        {
          type: 'category',
          label: 'Chapter 7 Exercises',
          items: [
            'module2/chapter7_exercises',
          ],
        },
        'module2/chapter8_photorealistic_rendering',
        {
          type: 'category',
          label: 'Chapter 8 Exercises',
          items: [
            'module2/chapter8_exercises',
          ],
        },
        'module2/chapter9_domain_randomization',
        {
          type: 'category',
          label: 'Chapter 9 Exercises',
          items: [
            'module2/chapter9_exercises',
          ],
        },
        'module2/chapter10_closing_sim_loop',
        {
          type: 'category',
          label: 'Chapter 10 Exercises',
          items: [
            'module2/chapter10_exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain – NVIDIA Isaac Platform',
      items: [
        'module3/intro',
        'module3/chapter11_simulation_2025',
        'module3/chapter12_ros2_fundamentals',
        'module3/chapter13_advanced_navigation',
        'module3/chapter14_reinforcement_learning',
        'module3/chapter15_sim_to_real_transfer',
        'module3/summary'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Models – From Voice to Physical Action',
      items: [
        'module4/intro',
        'module4/chapter16_vla_revolution',
        {
          type: 'category',
          label: 'Chapter 16 Exercises',
          items: [
            'module4/chapter16_exercises',
          ],
        },
        'module4/chapter17_fine_tuning',
        {
          type: 'category',
          label: 'Chapter 17 Exercises',
          items: [
            'module4/chapter17_exercises',
          ],
        },
        'module4/chapter18_voice_action_pipeline',
        {
          type: 'category',
          label: 'Chapter 18 Exercises',
          items: [
            'module4/chapter18_exercises',
          ],
        },
        'module4/chapter19_multi_modal_foundations',
        {
          type: 'category',
          label: 'Chapter 19 Exercises',
          items: [
            'module4/chapter19_exercises',
          ],
        },
        'module4/chapter20_sim_to_real_transfer',
        {
          type: 'category',
          label: 'Chapter 20 Exercises',
          items: [
            'module4/chapter20_exercises',
          ],
        },
        'module4/summary'
      ],
    },
  ],
};