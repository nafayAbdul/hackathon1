const {themes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'The Definitive 2025 Practitioner\'s Book',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-book-url.github.io/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'Abdul-Nafay-331', // Usually your GitHub org/user name.
  projectName: 'Hackathon-I-Create-a-Textbook-for-Teaching-Physical-AI-Humanoid-Robotics-Course', // Usually your repo name.
  deploymentBranch: 'gh-pages', // The branch to deploy to

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  customFields: {
    // This correctly reads the variable during the Vercel build process
    apiBaseUrl: process.env.REACT_APP_API_URL ||
                (process.env.HUGGINGFACE_SPACE_URL ?
                 process.env.HUGGINGFACE_SPACE_URL :
                 'https://nafaywork5523-physical-chatbot.hf.space'),
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/nafayAbdul/hackathon1/tree/main/docs/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    // Plugin to wrap the layout with the chat widget
    async function myPlugin() {
      return {
        name: 'chat-widget-plugin',
        configureWebpack(config, isServer, utils) {
          return {
            resolve: {
              alias: {
                '@theme/ChatWidget': './src/theme/ChatWidget.js',
              },
            },
          };
        },
      };
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/physical-ai-social-card.jpg',
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/favicon.ico',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            href: 'https://github.com/nafayAbdul/hackathon1',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System',
                to: '/docs/module1/intro',
              },
              {
                label: 'Module 2: Simulation Integration – The Digital Twin',
                to: '/docs/module2/intro',
              },
              {
                label: 'Module 3: Simulation & Reinforcement Learning',
                to: '/docs/module3/intro',
              },
              {
                label: 'Module 4: Vision-Language-Action Models',
                to: '/docs/module4/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/ros',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/ros_org',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/nafayAbdul/hackathon1',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
        additionalLanguages: ['python', 'bash'],
      },
    }),

  stylesheets: [
    'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css',
    'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=Space+Grotesk:wght@600;700&display=swap',
    'https://fonts.googleapis.com/css2?family=Orbitron:wght@500;600;700&display=swap',
    'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600&display=swap',
  ],

  headTags: [
    // Content Security Policy for security
    {
      tagName: 'meta',
      attributes: {
        name: 'Content-Security-Policy',
        content: "default-src 'self'; script-src 'self' 'unsafe-inline' https://www.google-analytics.com https://www.googletagmanager.com; style-src 'self' 'unsafe-inline' https://fonts.googleapis.com https://cdnjs.cloudflare.com; font-src 'self' https://fonts.gstatic.com https://cdnjs.cloudflare.com; img-src 'self' data: https:; connect-src 'self' https://www.google-analytics.com; frame-ancestors 'none';",
      },
    },
  ],
};

module.exports = config;
