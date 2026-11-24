import { Component, type ReactNode } from "react";

import styles from "./states.module.css";

type ErrorBoundaryProps = {
  children: ReactNode;
};

type ErrorBoundaryState = {
  hasError: boolean;
};

export class ErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  state: ErrorBoundaryState = { hasError: false };

  static getDerivedStateFromError(): ErrorBoundaryState {
    return { hasError: true };
  }

  reset = (): void => {
    this.setState({ hasError: false });
  };

  render(): ReactNode {
    if (this.state.hasError) {
      return (
        <section className={styles.alert} role="alert">
          <h1 className={styles.heading}>Something went wrong</h1>
          <p>The review interface hit an unexpected error.</p>
          <button className={styles.button} type="button" onClick={this.reset}>
            Try again
          </button>
        </section>
      );
    }
    return this.props.children;
  }
}
