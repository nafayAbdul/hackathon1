import React from 'react';
import clsx from 'clsx';
import './CTAButton.css';

/**
 * Call-to-action button with special effects
 * @param {Object} props
 * @param {ReactNode} props.children - Button content
 * @param {string} [props.href] - Link destination (optional)
 * @param {Function} [props.onClick] - Click handler (optional)
 * @param {string} [props.variant] - Button style variant (optional)
 * @param {string} [props.className] - Additional CSS classes
 */
const CTAButton = ({ children, href, onClick, variant, className, ...props }) => {
  const buttonClasses = clsx(
    'cta-button',
    variant && `cta-button--${variant}`,
    className
  );

  if (href) {
    return (
      <a
        href={href}
        className={buttonClasses}
        onClick={onClick}
        role="button"
        tabIndex="0"
        {...props}
      >
        {children}
      </a>
    );
  }

  return (
    <button
      className={buttonClasses}
      onClick={onClick}
      {...props}
    >
      {children}
    </button>
  );
};

export default CTAButton;