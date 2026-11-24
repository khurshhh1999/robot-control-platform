import styles from "./states.module.css";

type ApiDownStateProps = {
  onRetry: () => void;
};

export function ApiDownState({ onRetry }: ApiDownStateProps) {
  return (
    <section className={styles.alert} role="alert">
      <h2 className={styles.heading}>API unavailable</h2>
      <p>The review interface cannot reach the API.</p>
      <button className={styles.button} type="button" onClick={onRetry}>
        Try again
      </button>
    </section>
  );
}
