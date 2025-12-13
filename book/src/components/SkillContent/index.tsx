import React, { ReactNode } from 'react';
import { useSkillLevel } from '../../context/SkillLevelContext';

type SupportedSkillLevel = 'Beginner' | 'Intermediate' | 'Expert';

interface SkillContentProps {
  level: SupportedSkillLevel;
  children: ReactNode;
}

const skillLevelOrder: Record<SupportedSkillLevel, number> = {
  Beginner: 0,
  Intermediate: 1,
  Expert: 2,
};

export default function SkillContent({ level, children }: SkillContentProps): JSX.Element | null {
  const { skillLevel: currentSkillLevel } = useSkillLevel();

  // Only render content if the user's selected skill level is at or above
  // the level required by this content block.
  if (skillLevelOrder[currentSkillLevel] >= skillLevelOrder[level]) {
    return <>{children}</>;
  }

  return null;
}
