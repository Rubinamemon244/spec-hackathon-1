import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useThemeConfig} from '@docusaurus/theme-common';
import {FooterLinkItem, useFooterLinks} from '@theme/Footer/FooterLinkItem';
import styles from './Footer.module.css';

function FooterLinkItemComponent(props) {
  return (
    <FooterLinkItem {...props} />
  );
}

function Footer() {
  const {footer} = useThemeConfig();
  const footerLinks = useFooterLinks();
  const {copyright, links, logo, style} = footer || {};

  if (!footer) {
    return null;
  }

  return (
    <footer
      className={clsx('footer', {
        'footer--dark': style === 'dark',
      })}>
      {links && links.length > 0 && (
        <div className="container">
          <div className="row footer__links">
            {links.map((linkSection, i) => (
              <div key={i} className="col footer__col">
                {linkSection.title != null ? (
                  <h4 className="footer__title">{linkSection.title}</h4>
                ) : null}
                {linkSection.items != null &&
                Array.isArray(linkSection.items) &&
                linkSection.items.length > 0 ? (
                  <ul className="footer__items">
                    {linkSection.items.map((item, key) =>
                      item.html ? (
                        <li
                          key={key}
                          className="footer__item"
                          dangerouslySetInnerHTML={{
                            __html: item.html,
                          }}
                        />
                      ) : (
                        <li key={item.href || item.to} className="footer__item">
                          <FooterLinkItemComponent {...item} />
                        </li>
                      ),
                    )}
                  </ul>
                ) : null}
              </div>
            ))}
          </div>
        </div>
      )}
      <div className="footer__bottom">
        <div className="container">
          <div className="footer__bottom-content">
            {copyright ? (
              <div className="footer__copyright">
                {copyright.startsWith('Copyright') ? (
                  <>
                    {copyright.substring(0, copyright.indexOf('Built by'))}
                    {copyright.substring(copyright.indexOf('Built by'))}
                    <span className="rubina-memon">Rubina Memon 💙🤖</span>
                  </>
                ) : (
                  copyright
                )}
              </div>
            ) : null}
          </div>
        </div>
      </div>
    </footer>
  );
}

export default Footer;