import React from 'react';
import clsx from 'clsx';
import HomepageModuleCard from './HomepageModuleCard';
import styles from './HomepageModulesGrid.module.css';

/**
 * HomepageModulesGrid Component
 * A grid component for displaying the 6 course modules
 * @param {Object} props - Component properties
 * @param {Array} props.modules - Array of module objects to display
 * @param {string} props.title - Optional title for the grid section
 * @param {string} props.description - Optional description for the grid section
 */
function HomepageModulesGrid({ modules = [], title, description }) {
  return (
    <section className={styles.modulesGridSection}>
      {(title || description) && (
        <div className={styles.modulesGridHeader}>
          {title && (
            <h2 className={styles.modulesGridTitle}>{title}</h2>
          )}
          {description && (
            <p className={styles.modulesGridDescription}>{description}</p>
          )}
        </div>
      )}

      <div className={styles.modulesGrid}>
        {modules.map((module, index) => (
          <div key={index} className={styles.modulesGridItem}>
            <HomepageModuleCard
              title={module.title}
              description={module.description}
              href={module.href}
              moduleNumber={module.moduleNumber}
              image={module.image}
              category={module.category}
              highlighted={module.highlighted}
            />
          </div>
        ))}
      </div>
    </section>
  );
}

export default HomepageModulesGrid;