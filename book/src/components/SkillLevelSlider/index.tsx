import React, { useEffect } from 'react';
import styles from './styles.module.css';
import { useSkillLevel } from '../../context/SkillLevelContext'; // Adjust path as necessary

export default function SkillLevelSlider(): JSX.Element {
  const { skillLevel, setSkillLevel } = useSkillLevel();

  // Convert string skill level to a number for the slider's value
  const skillLevelToNumber = (level: string) => {
    switch (level) {
      case 'Beginner': return 0;
      case 'Intermediate': return 1;
      case 'Expert': return 2;
      default: return 0;
    }
  };

  const numberToSkillLevel = (num: number): 'Beginner' | 'Intermediate' | 'Expert' => {
    switch (num) {
      case 0: return 'Beginner';
      case 1: return 'Intermediate';
      case 2: return 'Expert';
      default: return 'Beginner';
    }
  };

  const handleSliderChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const newLevelNumber = parseInt(event.target.value, 10);
    setSkillLevel(numberToSkillLevel(newLevelNumber));
  };

  return (
    <div className={styles.sliderContainer}>
      <label htmlFor="skill-slider">Skill Level: {skillLevel}</label>
      <input
        type="range"
        id="skill-slider"
        name="skill-slider"
        min="0"
        max="2"
        step="1"
        value={skillLevelToNumber(skillLevel)}
        onChange={handleSliderChange}
        className={styles.slider}
      />
      <div className={styles.labels}>
        <span>Beginner</span>
        <span>Intermediate</span>
        <span>Expert</span>
      </div>
    </div>
  );
}
