import React from 'react';
import styles from './IsaacCard.module.css';

/**
 * IsaacTutorialLayout Component
 * A layout component for Isaac tutorials with common structure
 */
export const IsaacTutorialLayout = ({ title, children, subtitle }) => {
  return (
    <div className={styles.isaacTutorial}>
      <div className={styles.isaacHeader}>
        <h1>{title}</h1>
        {subtitle && <p>{subtitle}</p>}
      </div>
      <div className={styles.isaacSection}>
        {children}
      </div>
    </div>
  );
};

/**
 * IsaacSection Component
 * A section component for organizing Isaac tutorial content
 */
export const IsaacSection = ({ title, children, highlight = false }) => {
  const sectionClass = highlight
    ? `${styles.isaacSection} ${styles.isaacHighlight}`
    : styles.isaacSection;

  return (
    <div className={sectionClass}>
      {title && <h2>{title}</h2>}
      {children}
    </div>
  );
};

/**
 * IsaacCodeBlock Component
 * A styled code block component for Isaac examples
 */
export const IsaacCodeBlock = ({ children, language = 'bash' }) => {
  return (
    <pre className={styles.isaacCodeBlock}>
      <code className={`language-${language}`}>
        {children}
      </code>
    </pre>
  );
};