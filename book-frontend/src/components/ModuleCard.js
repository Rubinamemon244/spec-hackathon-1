import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './ModuleCard.module.css';

/**
 * ModuleCard Component
 * A card component for displaying course modules
 * @param {Object} props - Component properties
 * @param {string} props.title - Title of the module
 * @param {string} props.description - Description of the module
 * @param {string} props.href - Link to the module page
 * @param {string} props.icon - Optional icon component or character
 * @param {string} props.category - Category or tag for the module
 * @param {string} props.level - Difficulty level (beginner, intermediate, advanced)
 * @param {Array} props.tags - Array of tags for the module
 * @param {boolean} props.highlighted - Whether to highlight the card
 * @param {string} props.variant - Card variant (default, primary, secondary)
 */
function ModuleCard({
  title,
  description,
  href,
  icon,
  category,
  level,
  tags = [],
  highlighted = false,
  variant = 'default'
}) {
  const cardClasses = clsx(
    styles.moduleCard,
    styles[`variant-${variant}`],
    {
      [styles.highlighted]: highlighted,
    }
  );

  const getLevelColor = (level) => {
    switch (level?.toLowerCase()) {
      case 'beginner':
        return styles.levelBeginner;
      case 'intermediate':
        return styles.levelIntermediate;
      case 'advanced':
        return styles.levelAdvanced;
      default:
        return '';
    }
  };

  return (
    <div className={cardClasses}>
      <Link to={href} className={styles.moduleCardLink}>
        <div className={styles.moduleCardContent}>
          {icon && (
            <div className={styles.moduleCardIcon}>
              {icon}
            </div>
          )}
          {category && (
            <span className={styles.moduleCardCategory}>
              {category}
            </span>
          )}
          <Heading as="h3" className={styles.moduleCardTitle}>
            {title}
          </Heading>
          <p className={styles.moduleCardDescription}>
            {description}
          </p>
          {level && (
            <span className={clsx(styles.moduleCardLevel, getLevelColor(level))}>
              {level}
            </span>
          )}
          {tags.length > 0 && (
            <div className={styles.moduleCardTags}>
              {tags.map((tag, index) => (
                <span key={index} className={styles.moduleCardTag}>
                  {tag}
                </span>
              ))}
            </div>
          )}
        </div>
      </Link>
    </div>
  );
}

export default ModuleCard;