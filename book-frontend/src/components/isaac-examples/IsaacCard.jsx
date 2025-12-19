import React from 'react';
import styles from './IsaacCard.module.css';

/**
 * IsaacCard Component
 * A reusable card component for displaying Isaac-related content
 */
export const IsaacCard = React.memo(({ title, children, type = 'info' }) => {
  return (
    <div className={`${styles.card} ${styles[type]}`} role="region" aria-labelledby="isaac-card-title">
      <h3 id="isaac-card-title" className={styles.title}>{title}</h3>
      <div className={styles.content} tabIndex="0" aria-label={`Content for ${title}`}>
        {children}
      </div>
    </div>
  );
});

/**
 * IsaacExample Component
 * Component for displaying code examples with Isaac-specific styling
 */
export const IsaacExample = React.memo(({ title, children, language = 'bash' }) => {
  const codeId = `code-${Math.random().toString(36).substr(2, 9)}`;
  return (
    <div className={styles.example} role="region" aria-labelledby={codeId}>
      <h4 id={codeId} className={styles.exampleTitle}>{title}</h4>
      <pre className={styles.exampleCode}>
        <code className={`language-${language}`} aria-label={`${title} code example`}>
          {children}
        </code>
      </pre>
    </div>
  );
});

/**
 * IsaacNote Component
 * Component for displaying important notes in Isaac tutorials
 */
export const IsaacNote = React.memo(({ children }) => {
  return (
    <div className={styles.note} role="note" aria-label="Important note">
      <div className={styles.noteIcon} aria-hidden="true">💡</div>
      <div className={styles.noteContent} tabIndex="0">
        {children}
      </div>
    </div>
  );
});