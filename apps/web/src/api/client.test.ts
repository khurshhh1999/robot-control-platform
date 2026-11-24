import { describe, expect, it, vi } from "vitest";

import { createApiClient, IDEMPOTENCY_KEY_HEADER, REQUEST_ID_HEADER } from "./client";
import { ApiProblemError, ApiTransportError } from "./errors";

function jsonResponse(body: unknown, status: number, headers: Record<string, string> = {}): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json", ...headers },
  });
}

function requestHeaders(fetchImpl: {
  mock: { calls: ReadonlyArray<readonly unknown[]> };
}): Headers {
  const init = fetchImpl.mock.calls[0]?.[1];
  if (typeof init !== "object" || init === null || !("headers" in init)) {
    return new Headers();
  }
  const headers = init.headers;
  if (
    headers instanceof Headers ||
    Array.isArray(headers) ||
    (typeof headers === "object" && headers !== null)
  ) {
    return new Headers(headers as HeadersInit);
  }
  return new Headers();
}

describe("api client", () => {
  it("parses JSON and returns the response request id", async () => {
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, _init?: RequestInit) =>
      jsonResponse({ status: "ok" }, 200, { [REQUEST_ID_HEADER]: "req-live" }),
    );
    const client = createApiClient({ fetchImpl });
    const result = await client.requestJson("/health/live", {
      method: "GET",
      requestId: "caller-1",
    });
    expect(result).toEqual({
      kind: "json",
      status: 200,
      requestId: "req-live",
      data: { status: "ok" },
    });
    expect(requestHeaders(fetchImpl).get(REQUEST_ID_HEADER)).toBe("caller-1");
  });

  it("omits an invalid request id and still reads the server id", async () => {
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, _init?: RequestInit) =>
      jsonResponse({ status: "ok" }, 200, { [REQUEST_ID_HEADER]: "generated-id" }),
    );
    const client = createApiClient({ fetchImpl });
    const result = await client.requestJson("/health/live", {
      requestId: "bad id",
    });
    expect(result.requestId).toBe("generated-id");
    expect(requestHeaders(fetchImpl).get(REQUEST_ID_HEADER)).toBeNull();
  });

  it("throws problem details without retrying inside fetch", async () => {
    const fetchImpl = vi.fn(async () =>
      new Response(
        JSON.stringify({
          type: "urn:robot-control-platform:problem:RESOURCE_NOT_FOUND",
          title: "Resource Not Found",
          status: 404,
          detail: "The requested resource was not found",
          code: "RESOURCE_NOT_FOUND",
        }),
        {
          status: 404,
          headers: {
            "Content-Type": "application/problem+json",
            [REQUEST_ID_HEADER]: "req-404",
          },
        },
      ),
    );
    const client = createApiClient({ fetchImpl });
    await expect(client.requestJson("/api/v1/trials/missing")).rejects.toMatchObject({
      name: "ApiProblemError",
      status: 404,
      code: "RESOURCE_NOT_FOUND",
      requestId: "req-404",
      method: "GET",
    });
    expect(fetchImpl).toHaveBeenCalledTimes(1);
  });

  it("hides unexpected error bodies", async () => {
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, _init?: RequestInit) =>
      new Response(JSON.stringify({ password: "secret-value" }), {
        status: 500,
        headers: { "Content-Type": "text/plain" },
      }),
    );
    const client = createApiClient({ fetchImpl });
    try {
      await client.requestJson("/health/live");
      throw new Error("expected a problem");
    } catch (error) {
      expect(error).toBeInstanceOf(ApiProblemError);
      expect(error instanceof Error ? error.message : "").not.toContain("secret-value");
    }
  });

  it("returns content bytes, media type, length, and etag", async () => {
    const bytes = new Uint8Array([1, 2, 3, 4]);
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, _init?: RequestInit) =>
      new Response(bytes, {
        status: 200,
        headers: {
          "Content-Type": "image/png",
          "Content-Length": "4",
          ETag: '"abc"',
          [REQUEST_ID_HEADER]: "req-bytes",
        },
      }),
    );
    const client = createApiClient({ fetchImpl });
    const result = await client.requestContent("/api/v1/artifacts/one/content");
    expect(result.kind).toBe("content");
    expect(result.mediaType).toBe("image/png");
    expect(result.contentLength).toBe(4);
    expect(result.etag).toBe('"abc"');
    expect(result.requestId).toBe("req-bytes");
    expect(new Uint8Array(result.body)).toEqual(bytes);
  });

  it("rethrows abort errors", async () => {
    const controller = new AbortController();
    controller.abort();
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, init?: RequestInit) => {
      if (init?.signal?.aborted) {
        throw new DOMException("The operation was aborted.", "AbortError");
      }
      return jsonResponse({ status: "ok" }, 200);
    });
    const client = createApiClient({ fetchImpl });
    await expect(
      client.requestJson("/health/live", { signal: controller.signal }),
    ).rejects.toMatchObject({ name: "AbortError" });
  });

  it("wraps network failures and sends a safe idempotency key", async () => {
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, _init?: RequestInit) => {
      throw new TypeError("Failed to fetch");
    });
    const client = createApiClient({ baseUrl: "http://127.0.0.1:8000/", fetchImpl });
    await expect(
      client.requestJson("/api/v1/experiments/one/runs", {
        method: "POST",
        idempotencyKey: "start-run-1",
        requestId: "req-post",
      }),
    ).rejects.toBeInstanceOf(ApiTransportError);
    expect(String(fetchImpl.mock.calls[0]?.[0])).toBe(
      "http://127.0.0.1:8000/api/v1/experiments/one/runs",
    );
    const headers = requestHeaders(fetchImpl);
    expect(headers.get(IDEMPOTENCY_KEY_HEADER)).toBe("start-run-1");
    expect(headers.get(REQUEST_ID_HEADER)).toBe("req-post");
  });

  it("does not send an idempotency key that could break the header", async () => {
    const fetchImpl = vi.fn(async (_input: RequestInfo | URL, _init?: RequestInit) =>
      jsonResponse({ status: "ok" }, 200),
    );
    const client = createApiClient({ fetchImpl });
    await client.requestJson("/health/live", { idempotencyKey: "bad\nkey" });
    expect(requestHeaders(fetchImpl).get(IDEMPOTENCY_KEY_HEADER)).toBeNull();
  });
});
