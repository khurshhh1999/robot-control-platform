import { Link } from "react-router";

import styles from "./states.module.css";

export function NotFoundPage() {
  return (
    <section className={styles.stack}>
      <h1 className={styles.heading}>Page not found</h1>
      <p>That address is not part of the review interface.</p>
      <Link className={styles.link} to="/">
        Back to overview
      </Link>
    </section>
  );
}
