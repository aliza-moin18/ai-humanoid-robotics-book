import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics - Book',
  tagline: 'From Simulated Brains to Embodied Intelligence',
  favicon: 'img/favicon.ico',
  future: {
    v4: true,
  },
  url: 'https://aliza-moin18.github.io',
  baseUrl: '/ai-humanoid-robotics-book/', 
  organizationName: 'aliza-moin18',
  projectName: 'ai-humanoid-robotics-book',
  
  onBrokenLinks: 'warn',  
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/aliza-moin18/ai-humanoid-robotics-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/aliza-moin18/ai-humanoid-robotics-book/tree/main/',
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
    image: 'img/robo.png',
    colorMode: {
      respectPrefersColorScheme: true,
      defaultMode: 'dark',
    },
    navbar: {
      title: 'AI Robotics',
      logo: {
        alt: 'AI Robotics Logo',
        src: 'img/robo.png',
        height: 60,
      },
      items: [
        {
          to: '/docs/module-1-ros2-fundamentals',
          label: 'Read Book',
          position: 'left',
        },
        {
          href: 'https://github.com/aliza-moin18/ai-humanoid-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,

  stylesheets: [
    'https://fonts.googleapis.com/css2?family=Montserrat:wght@400;500;700&display=swap',
    {
      href: 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.0/css/all.min.css',
      type: 'text/css',
      integrity: 'sha512-SnH5WK+bZxgPHs44uWIX+LLJAJ9/2PkPKZ5QiAj6Ta86w+fsb2TkcmfRyVX3pBnMFcV7oQPJkl9QevSCWr3W6A==',
      crossorigin: 'anonymous',
    },
  ],
};

export default config;
