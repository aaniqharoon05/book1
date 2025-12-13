import React from 'react';
import { SkillLevelProvider } from '../context/SkillLevelContext';

export default function Root({children}) {
  return (
    <SkillLevelProvider>
      {children}
    </SkillLevelProvider>
  );
}