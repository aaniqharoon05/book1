import React, {type ReactNode} from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type {WrapperProps} from '@docusaurus/types';

// Import the SkillLevelProvider and SkillLevelSlider
import { SkillLevelProvider } from '../../context/SkillLevelContext';
import SkillLevelSlider from '../../components/SkillLevelSlider'; // Assuming index.tsx is default export

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): ReactNode {
  return (
    // Wrap the DocItem with the SkillLevelProvider
    <SkillLevelProvider>
      {/* Render the SkillLevelSlider above the main document content */}
      <SkillLevelSlider />
      <DocItem {...props} />
    </SkillLevelProvider>
  );
}
