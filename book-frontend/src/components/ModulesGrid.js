import React from 'react';
import clsx from 'clsx';
import ModuleCard from './ModuleCard';
import styles from './ModulesGrid.module.css';

/**
 * ModulesGrid Component
 * A responsive grid component for displaying multiple ModuleCard components
 * @param {Object} props - Component properties
 * @param {Array} props.modules - Array of module objects to display
 * @param {string} props.gridSize - Grid size (small, medium, large, auto)
 * @param {string} props.layout - Layout type (grid, list, masonry)
 * @param {boolean} props.showFilter - Whether to show filter controls
 * @param {Array} props.categories - Available categories for filtering
 * @param {string} props.title - Optional title for the grid section
 * @param {string} props.description - Optional description for the grid section
 */
function ModulesGrid({
  modules = [],
  gridSize = 'medium',
  layout = 'grid',
  showFilter = false,
  categories = [],
  title,
  description
}) {
  const gridClasses = clsx(
    styles.modulesGrid,
    styles[`gridSize-${gridSize}`],
    styles[`layout-${layout}`]
  );

  const [selectedCategory, setSelectedCategory] = React.useState('all');

  const filteredModules = selectedCategory === 'all'
    ? modules
    : modules.filter(module =>
        module.category?.toLowerCase() === selectedCategory.toLowerCase()
      );

  const uniqueCategories = [...new Set(modules.map(module => module.category))].filter(Boolean);

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

      {showFilter && uniqueCategories.length > 0 && (
        <div className={styles.modulesGridFilters}>
          <button
            className={clsx(
              styles.filterButton,
              selectedCategory === 'all' && styles.filterButtonActive
            )}
            onClick={() => setSelectedCategory('all')}
          >
            All
          </button>
          {uniqueCategories.map((category, index) => (
            <button
              key={index}
              className={clsx(
                styles.filterButton,
                selectedCategory === category && styles.filterButtonActive
              )}
              onClick={() => setSelectedCategory(category)}
            >
              {category}
            </button>
          ))}
        </div>
      )}

      <div className={gridClasses}>
        {filteredModules.map((module, index) => (
          <ModuleCard
            key={index}
            title={module.title}
            description={module.description}
            href={module.href}
            icon={module.icon}
            category={module.category}
            level={module.level}
            tags={module.tags}
            highlighted={module.highlighted}
            variant={module.variant}
          />
        ))}
      </div>

      {filteredModules.length === 0 && (
        <div className={styles.noResults}>
          No modules found for the selected category.
        </div>
      )}
    </section>
  );
}

export default ModulesGrid;