import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './SectionHeader.module.css';

/**
 * SectionHeader Component
 * A consistent header component for sections
 * @param {Object} props - Component properties
 * @param {string} props.title - Main title of the section
 * @param {string} props.subtitle - Subtitle or description
 * @param {string} props.description - Detailed description
 * @param {string} props.align - Text alignment (left, center, right)
 * @param {string} props.size - Header size (small, medium, large)
 * @param {boolean} props.divider - Whether to show a divider line
 * @param {string} props.variant - Header variant (default, primary, secondary)
 * @param {boolean} props.withIcon - Whether to show an icon
 * @param {React.ReactNode} props.icon - Icon component to display
 * @param {string} props.iconPosition - Position of icon (left, right, top)
 */
function SectionHeader({
  title,
  subtitle,
  description,
  align = 'center',
  size = 'medium',
  divider = true,
  variant = 'default',
  withIcon = false,
  icon,
  iconPosition = 'left'
}) {
  const headerClasses = clsx(
    styles.sectionHeader,
    styles[`align-${align}`],
    styles[`size-${size}`],
    styles[`variant-${variant}`],
    {
      [styles.withDivider]: divider,
    }
  );

  const iconClasses = clsx(
    styles.sectionIcon,
    styles[`iconPosition-${iconPosition}`]
  );

  const shouldShowIcon = withIcon && icon;

  return (
    <header className={headerClasses}>
      <div className={styles.sectionHeaderContent}>
        {shouldShowIcon && iconPosition === 'top' && (
          <div className={iconClasses}>
            {icon}
          </div>
        )}
        <div className={styles.sectionHeaderText}>
          {subtitle && (
            <div className={styles.sectionSubtitle}>
              {subtitle}
            </div>
          )}
          {title && (
            <Heading as="h2" className={styles.sectionTitle}>
              {title}
            </Heading>
          )}
          {description && (
            <p className={styles.sectionDescription}>
              {description}
            </p>
          )}
        </div>
        {shouldShowIcon && iconPosition !== 'top' && (
          <div className={iconClasses}>
            {icon}
          </div>
        )}
      </div>
      {divider && (
        <div className={styles.sectionDivider} />
      )}
    </header>
  );
}

export default SectionHeader;