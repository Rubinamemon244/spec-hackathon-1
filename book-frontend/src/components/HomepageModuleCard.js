import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './HomepageModuleCard.module.css';

/**
 * HomepageModuleCard Component
 * A card component for displaying course modules with background images
 * @param {Object} props - Component properties
 * @param {string} props.title - Title of the module
 * @param {string} props.description - Description of the module
 * @param {string} props.href - Link to the module page
 * @param {string} props.moduleNumber - Module number
 * @param {string} props.image - Background image URL
 * @param {string} props.category - Category for the module
 * @param {boolean} props.highlighted - Whether to highlight the card
 */
function HomepageModuleCard({
  title,
  description,
  href,
  moduleNumber,
  image,
  category,
  highlighted = false
}) {
  const cardClasses = clsx(
    styles.moduleCard,
    {
      [styles.highlighted]: highlighted,
    }
  );

  const cardStyle = image ? {
    backgroundImage: `url(${image})`,
  } : {};

  return (
    <Link to={href} className={styles.moduleCardLink}>
      <div className={cardClasses} style={cardStyle}>
        <div className={styles.moduleCardContent}>
          <div className={styles.moduleCardHeader}>
            <span className={styles.moduleNumber}>
              Module {moduleNumber}
            </span>
            {category && (
              <span className={styles.moduleCategory}>
                {category}
              </span>
            )}
          </div>
          <Heading as="h3" className={styles.moduleCardTitle}>
            {title}
          </Heading>
          <p className={styles.moduleCardDescription}>
            {description}
          </p>
          <div className={styles.moduleCardFooter}>
            <span className={styles.learnMore}>
              Learn More →
            </span>
          </div>
        </div>
      </div>
    </Link>
  );
}

export default HomepageModuleCard;