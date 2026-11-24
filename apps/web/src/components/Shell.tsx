import type { MouseEvent } from "react";
import { NavLink, Outlet } from "react-router";

import { productTitle } from "../appName";
import { ErrorBoundary } from "./ErrorBoundary";
import styles from "./Shell.module.css";

function focusMain(event: MouseEvent<HTMLAnchorElement>): void {
  const main = document.getElementById("main-content");
  if (!(main instanceof HTMLElement)) {
    return;
  }
  event.preventDefault();
  main.focus();
}

export function RootLayout() {
  return (
    <ErrorBoundary>
      <div className={styles.page}>
        <a className={styles.skipLink} href="#main-content" onClick={focusMain}>
          Skip to content
        </a>
        <header className={styles.header}>
          <p className={styles.product}>{productTitle}</p>
          <nav aria-label="Primary">
            <ul className={styles.navList}>
              <li>
                <NavLink className={styles.navLink} to="/" end>
                  Overview
                </NavLink>
              </li>
            </ul>
          </nav>
        </header>
        <main id="main-content" tabIndex={-1} className={styles.main}>
          <ErrorBoundary>
            <Outlet />
          </ErrorBoundary>
        </main>
      </div>
    </ErrorBoundary>
  );
}
