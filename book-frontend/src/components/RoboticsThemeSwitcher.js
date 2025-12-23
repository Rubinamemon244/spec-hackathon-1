import React, { useState, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

const RoboticsThemeSwitcher = () => {
  const { colorMode, setColorMode } = useColorMode();
  const [currentTheme, setCurrentTheme] = useState('default');

  // Apply theme to document element
  useEffect(() => {
    const root = document.documentElement;

    // Remove all theme classes
    root.classList.remove('dark-ai-lab', 'robotics-steel', 'neural-horizon');

    // Apply current theme class
    if (currentTheme !== 'default') {
      root.setAttribute('data-theme', currentTheme);
      root.classList.add(currentTheme);
    } else {
      root.removeAttribute('data-theme');
    }
  }, [currentTheme]);

  // Initialize theme from localStorage or default
  useEffect(() => {
    const savedTheme = localStorage.getItem('robotics-theme');
    if (savedTheme) {
      setCurrentTheme(savedTheme);
    }
  }, []);

  const handleThemeChange = (theme) => {
    setCurrentTheme(theme);
    localStorage.setItem('robotics-theme', theme);

    // Also set color mode appropriately
    if (theme !== 'default') {
      setColorMode('dark');
    }
  };

  const themes = [
    { value: 'default', label: 'Default Theme' },
    { value: 'dark-ai-lab', label: 'Dark AI Lab' },
    { value: 'robotics-steel', label: 'Robotics Steel' },
    { value: 'neural-horizon', label: 'Neural Horizon' },
    { value: 'quantum-cyberpunk', label: 'Quantum Cyberpunk' },
    { value: 'biomechanical', label: 'Biomechanical' }
  ];

  return (
    <div className="theme-switcher-container">
      <select
        value={currentTheme}
        onChange={(e) => handleThemeChange(e.target.value)}
        className="theme-selector"
        aria-label="Select color theme"
      >
        {themes.map(theme => (
          <option key={theme.value} value={theme.value}>
            {theme.label}
          </option>
        ))}
      </select>
      <style jsx>{`
        .theme-switcher-container {
          margin-left: 1rem;
        }

        .theme-selector {
          background-color: var(--ifm-background-surface-color, #f0f0f0);
          color: var(--ifm-color-content, #222);
          border: 1px solid var(--ifm-color-emphasis-300, #ddd);
          border-radius: 4px;
          padding: 0.25rem 0.5rem;
          font-size: 0.875rem;
          cursor: pointer;
        }

        .theme-selector:hover {
          border-color: var(--ifm-color-primary, #2563eb);
        }

        @media (max-width: 768px) {
          .theme-switcher-container {
            margin-left: 0.5rem;
          }
        }
      `}</style>
    </div>
  );
};

export default RoboticsThemeSwitcher;