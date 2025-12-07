import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// =======================================================
// AI-Native Physical AI & Humanoid Robotics Textbook
// Panaversity Hackathon Config
// =======================================================

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'AI-Native • Spec-Driven • Robotics & Embodied Intelligence',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // ✅ PRODUCTION SITE URL (GitHub Pages)
  url: 'https://madnan-github.github.io',

  // ✅ MUST MATCH YOUR REPO NAME
  baseUrl: '/ai-robo-learning/',

  // ✅ YOUR GITHUB USERNAME
  organizationName: 'madnan-github',

  // ✅ YOUR REPOSITORY NAME
  projectName: 'ai-robo-learning',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // ✅ Urdu Bonus Ready
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',

          // ✅ EDIT LINK FOR JUDGES
          editUrl:
            'https://github.com/madnan-github/learn-ai/tree/main/',

          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
        },

        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },

          editUrl:
            'https://github.com/madnan-github/learn-ai/tree/main/',

          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },

        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.png',

    colorMode: {
      respectPrefersColorScheme: true,
      defaultMode: 'dark',
    },

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Panaversity Robotics',
        src: 'img/logo.svg',
      },

      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: '📘 AI-Native Book',
        },

        {
          to: '/blog',
          label: '🧠 Research',
          position: 'left',
        },

        {
          to: '/docs/capstone',
          label: '🤖 Capstone',
          position: 'left',
        },

        {
          href: 'https://panaversity.org',
          label: '🎓 Panaversity',
          position: 'right',
        },

        {
          href: 'https://github.com/madnan-github/learn-ai',
          label: '⭐ GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',

      links: [
        {
          title: 'Book',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'ROS 2', to: '/docs/ros2' },
            { label: 'Gazebo & Unity', to: '/docs/simulation' },
            { label: 'NVIDIA Isaac', to: '/docs/isaac' },
            { label: 'VLA & GPT Robotics', to: '/docs/vla' },
          ],
        },

        {
          title: 'Hackathon',
          items: [
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
            {
              label: 'Spec-Kit Plus',
              href: 'https://github.com/panaversity/spec-kit-plus',
            },
            {
              label: 'Claude Code',
              href: 'https://www.anthropic.com',
            },
          ],
        },

        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discord.gg/panaversity',
            },
            {
              label: 'YouTube',
              href: 'https://youtube.com',
            },
          ],
        },

        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/madnan-github/learn-ai',
            },
          ],
        },
      ],

      copyright: `Copyright © ${new Date().getFullYear()} 
      Physical AI & Humanoid Robotics — AI-Native Textbook. 
      Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
