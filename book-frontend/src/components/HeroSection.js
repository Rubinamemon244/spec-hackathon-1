import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HeroSection.module.css';

/**
 * HeroSection Component
 * A modern hero section component for Docusaurus pages
 * @param {Object} props - Component properties
 * @param {string} props.title - Main title of the hero section
 * @param {string} props.subtitle - Subtitle or description
 * @param {string} props.backgroundColor - Background color theme (default, primary, dark)
 * @param {string} props.textColor - Text color theme (default, light, dark)
 * @param {Array} props.buttons - Array of button objects with text and href
 * @param {boolean} props.centered - Whether content should be centered
 */
function HeroSection({
  title,
  subtitle,
  backgroundColor = 'default',
  textColor = 'default',
  buttons = [],
  centered = true
}) {
  const containerClasses = clsx(
    styles.heroSection,
    styles[`bg-${backgroundColor}`],
    styles[`text-${textColor}`],
    {
      [styles.centered]: centered,
    }
  );

  return (
    <section className={containerClasses}>
      <div className="container">
        <div className="row">
          <div className={clsx('col', centered ? 'col--8 col--offset-2' : 'col--12')}>
            <div className={styles.heroContent}>
              <Heading as="h1" className={styles.heroTitle}>
                {title}
              </Heading>
              {subtitle && (
                <p className={styles.heroSubtitle}>
                  {subtitle}
                </p>
              )}
              {buttons.length > 0 && (
                <div className={styles.heroButtons}>
                  {buttons.map((button, index) => (
                    <a
                      key={index}
                      className={clsx('button button--outline button--primary', styles.heroButton)}
                      href={button.href}
                      target={button.external ? "_blank" : "_self"}
                      rel={button.external ? "noopener noreferrer" : undefined}
                    >
                      {button.text}
                    </a>
                  ))}
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default HeroSection;