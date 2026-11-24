import { useQuery } from "@tanstack/react-query";

import { useApiClient } from "../api/context";
import { readLiveHealth } from "../api/health";
import { ApiDownState } from "../components/ApiDownState";
import styles from "../components/states.module.css";

export function OverviewPage() {
  const client = useApiClient();
  const health = useQuery({
    queryKey: ["health", "live"],
    queryFn: ({ signal }) => readLiveHealth(client, signal),
  });

  let body = <p className={styles.muted}>The API did not report a usable status.</p>;
  if (health.isPending || (health.isFetching && health.isError)) {
    body = <p role="status">Checking whether the API is reachable.</p>;
  } else if (health.isError) {
    body = (
      <ApiDownState
        onRetry={() => {
          void health.refetch();
        }}
      />
    );
  } else if (health.data.status === "ok") {
    body = <p>The API is reachable.</p>;
  }

  return (
    <section className={styles.stack}>
      <h1 className={styles.heading}>Overview</h1>
      {body}
    </section>
  );
}
