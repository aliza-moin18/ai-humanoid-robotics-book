import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '96e'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'c1b'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '26d'),
            routes: [
              {
                path: '/docs/module-1-ros2-fundamentals/',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/', '7bc'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-1/',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-1/', '827'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-1/introduction-to-ros2',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-1/introduction-to-ros2', 'c3a'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-1/nodes-and-processes',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-1/nodes-and-processes', '9e7'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-1/services-and-parameters',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-1/services-and-parameters', 'fa0'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-1/topics-and-message-passing',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-1/topics-and-message-passing', 'd7e'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-2/',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-2/', '390'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-2/actions-for-complex-tasks',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-2/actions-for-complex-tasks', '7b6'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-3/',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-3/', '801'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-3/urdf-fundamentals',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-3/urdf-fundamentals', '1b9'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-4/',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-4/', '4a4'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-4/ros2-gazebo-integration',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-4/ros2-gazebo-integration', '450'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-5/',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-5/', 'bb6'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/chapter-5/designing-complete-applications',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/chapter-5/designing-complete-applications', '0ca'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-1-ros2-fundamentals/setup-guide',
                component: ComponentCreator('/docs/module-1-ros2-fundamentals/setup-guide', '63d'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1/',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1/', '464'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1/section-1-1-gazebo-installation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1/section-1-1-gazebo-installation', '0e3'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1/section-1-2-unity-installation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1/section-1-2-unity-installation', '832'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1/section-1-3-integration',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1/section-1-3-integration', '908'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1/section-1-4-verification',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1/section-1-4-verification', '03c'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2/',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2/', 'c5c'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2/section-2-1-gazebo-physics-concepts',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2/section-2-1-gazebo-physics-concepts', 'd32'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2/section-2-2-physical-properties',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2/section-2-2-physical-properties', '20e'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2/section-2-3-collision-detection',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2/section-2-3-collision-detection', '45d'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2/section-2-4-parameter-tuning',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2/section-2-4-parameter-tuning', '7d7'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3/',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3/', '3cc'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3/section-3-1-urdf-sdf-models',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3/section-3-1-urdf-sdf-models', '4b1'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3/section-3-2-physics-parameters',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3/section-3-2-physics-parameters', '3c1'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3/section-3-3-joint-link-configurations',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3/section-3-3-joint-link-configurations', '778'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3/section-3-4-environmental-physics',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3/section-3-4-environmental-physics', '8cf'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-4/section-4-1-lidar-simulation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-4/section-4-1-lidar-simulation', '8e0'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-4/section-4-2-imu-implementation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-4/section-4-2-imu-implementation', 'cdc'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-4/section-4-3-depth-sensor',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-4/section-4-3-depth-sensor', '4b3'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-4/section-4-4-sensor-data-processing',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-4/section-4-4-sensor-data-processing', '9b8'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-5/section-5-1-high-fidelity-visualization',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-5/section-5-1-high-fidelity-visualization', '630'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-5/section-5-2-rendering-techniques',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-5/section-5-2-rendering-techniques', 'c5c'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-5/section-5-3-human-robot-interaction',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-5/section-5-3-human-robot-interaction', 'cfd'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-1-sim-to-real-transfer-techniques',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-1-sim-to-real-transfer-techniques', '4b3'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-10-comprehensive-module-assessment',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-10-comprehensive-module-assessment', 'f34'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-11-technical-accuracy-review',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-11-technical-accuracy-review', 'c3d'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-12-reproducibility-verification',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-12-reproducibility-verification', '8f4'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-13-navigation-structure-validation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-13-navigation-structure-validation', 'ae8'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-14-apa-citation-compliance',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-14-apa-citation-compliance', 'ad8'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-15-constitutional-compliance-review',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-15-constitutional-compliance-review', 'b52'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-16-github-pages-deployment-preparation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-16-github-pages-deployment-preparation', 'be2'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-17-known-limitations-and-future-enhancements',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-17-known-limitations-and-future-enhancements', 'e04'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-2-domain-randomization-in-isaac-sim',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-2-domain-randomization-in-isaac-sim', '7d1'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-3-model-deployment-on-edge-hardware',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-3-model-deployment-on-edge-hardware', '1c9'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-4-lab-sim-to-real-transfer',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-4-lab-sim-to-real-transfer', '6b9'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-5-domain-randomization-techniques',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-5-domain-randomization-techniques', '86f'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-6-model-deployment-guidelines',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-6-model-deployment-guidelines', '419'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-7-performance-validation-methods',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-7-performance-validation-methods', '7bf'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-8-sim-to-real-transfer-assessment',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-8-sim-to-real-transfer-assessment', '205'),
                exact: true,
                sidebar: "docsSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-11/section-11-9-synthetic-to-real-training-data-quality',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-11/section-11-9-synthetic-to-real-training-data-quality', '3d2'),
                exact: true,
                sidebar: "docsSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
