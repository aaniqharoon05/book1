import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
      },
      items: [
        'chapter-1/lesson-1.1',
        'chapter-1/lesson-1.2',
        'chapter-1/lesson-1.3',
        'chapter-1/lesson-1.4',
        'chapter-1/lesson-1.5',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'generated-index',
      },
      items: [
        'chapter-2/lesson-2.1',
        'chapter-2/lesson-2.2',
        'chapter-2/lesson-2.3',
        'chapter-2/lesson-2.4',
        'chapter-2/lesson-2.5',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {
        type: 'generated-index',
      },
      items: [
        'chapter-3/lesson-3.1',
        'chapter-3/lesson-3.2',
        'chapter-3/lesson-3.3',
        'chapter-3/lesson-3.4',
        'chapter-3/lesson-3.5',
      ],
    },
  ],
};

export default sidebars;
