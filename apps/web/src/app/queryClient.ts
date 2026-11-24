import { QueryClient } from "@tanstack/react-query";

import { shouldRetryRead } from "../api/retry";

type QueryClientOptions = {
  retryDelay?: number;
};

export function createQueryClient(options: QueryClientOptions = {}): QueryClient {
  const retryDelay = options.retryDelay;
  return new QueryClient({
    defaultOptions: {
      queries: {
        retry: shouldRetryRead,
        retryDelay:
          retryDelay === undefined
            ? (failureCount: number) => Math.min(1000 * 2 ** failureCount, 5000)
            : retryDelay,
        refetchOnWindowFocus: false,
      },
      mutations: {
        retry: false,
      },
    },
  });
}
