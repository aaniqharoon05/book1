import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

type SkillLevel = 'Beginner' | 'Intermediate' | 'Expert';

interface SkillLevelContextType {
  skillLevel: SkillLevel;
  setSkillLevel: (level: SkillLevel) => void;
}

const SkillLevelContext = createContext<SkillLevelContextType | undefined>(undefined);

export const useSkillLevel = () => {
  const context = useContext(SkillLevelContext);
  if (!context) {
    throw new Error('useSkillLevel must be used within a SkillLevelProvider');
  }
  return context;
};

interface SkillLevelProviderProps {
  children: ReactNode;
}

export const SkillLevelProvider: React.FC<SkillLevelProviderProps> = ({ children }) => {
  const [skillLevel, setSkillLevelState] = useState<SkillLevel>(() => {
    if (typeof window !== 'undefined') {
      const savedLevel = localStorage.getItem('skillLevel') as SkillLevel;
      return savedLevel || 'Beginner';
    }
    return 'Beginner';
  });

  useEffect(() => {
    if (typeof window !== 'undefined') {
      localStorage.setItem('skillLevel', skillLevel);
    }
  }, [skillLevel]);

  const setSkillLevel = (level: SkillLevel) => {
    setSkillLevelState(level);
  };

  return (
    <SkillLevelContext.Provider value={{ skillLevel, setSkillLevel }}>
      {children}
    </SkillLevelContext.Provider>
  );
};
