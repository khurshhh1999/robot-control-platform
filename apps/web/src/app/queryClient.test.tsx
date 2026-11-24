import { QueryClientProvider, useMutation } from "@tanstack/react-query";
import { renderHook } from "@testing-library/react";
import { describe, expect, it } from "vitest";

import { ApiProblemError, ApiTransportError } from "../api/errors";
import { MAX_TRANSIENT_READ_RETRIES, shouldRetryRead } from "../api/retry";
import { createQueryClient } from "./queryClient";

function problem(method: string, status: number): ApiProblemError {
  return new ApiProblemError(
    {
      type: "urn:robot-control-platform:problem:DEPENDENCY_UNAVAILABLE",
      title: "Dependency Unavailable",
      status,
      detail: "Dependency unavailable",
      code: status >= 500 ? "DEPENDENCY_UNAVAILABLE" : "VALIDATION_ERROR",
    },
    { method, requestId: "req-retry" },
  );
}

describe("query retries", () => {
  it("does not retry client errors", () => {
    expect(shouldRetryRead(0, problem("GET", 404))).toBe(false);
    expect(shouldRetryRead(0, problem("GET", 422))).toBe(false);
  });

  it("retries a transient read only up to the bound", () => {
    const error = problem("GET", 503);
    expect(shouldRetryRead(0, error)).toBe(true);
    expect(shouldRetryRead(MAX_TRANSIENT_READ_RETRIES - 1, error)).toBe(true);
    expect(shouldRetryRead(MAX_TRANSIENT_READ_RETRIES, error)).toBe(false);
  });

  it("does not retry mutations or aborted reads", () => {
    expect(shouldRetryRead(0, problem("POST", 503))).toBe(false);
    expect(shouldRetryRead(0, new DOMException("aborted", "AbortError"))).toBe(false);
    const transport = new ApiTransportError("The API could not be reached.", {
      method: "GET",
      requestId: null,
    });
    expect(shouldRetryRead(0, transport)).toBe(true);
  });

  it("attempts a transient read once plus the retry bound", async () => {
    const client = createQueryClient({ retryDelay: 0 });
    let calls = 0;
    await expect(
      client.fetchQuery({
        queryKey: ["live"],
        queryFn: () => {
          calls += 1;
          throw problem("GET", 503);
        },
      }),
    ).rejects.toBeInstanceOf(ApiProblemError);
    expect(calls).toBe(1 + MAX_TRANSIENT_READ_RETRIES);
  });

  it("does not retry a failed mutation", async () => {
    const queryClient = createQueryClient({ retryDelay: 0 });
    expect(queryClient.getDefaultOptions().mutations?.retry).toBe(false);
    let calls = 0;
    const { result } = renderHook(
      () =>
        useMutation({
          mutationFn: () => {
            calls += 1;
            throw problem("POST", 503);
          },
        }),
      {
        wrapper: ({ children }) => (
          <QueryClientProvider client={queryClient}>{children}</QueryClientProvider>
        ),
      },
    );
    await expect(result.current.mutateAsync()).rejects.toBeInstanceOf(ApiProblemError);
    expect(calls).toBe(1);
  });
});
