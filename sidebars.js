/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs




 - render a sidebar for each doc of that group
 - provide next/previous navigation



 
 The sidebars can be generated from the filesystem, or explicitly defined here.

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
  ],
};