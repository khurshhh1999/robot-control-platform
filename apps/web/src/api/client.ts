import { ApiProblemError, ApiTransportError, isAbortError } from "./errors";
import { parseProblemDetail, unexpectedProblem } from "./problem";

export const REQUEST_ID_HEADER = "X-Request-Id";
export const IDEMPOTENCY_KEY_HEADER = "Idempotency-Key";

const REQUEST_ID_PATTERN = /^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$/;

export type HttpMethod = "GET" | "HEAD" | "POST" | "PUT" | "PATCH" | "DELETE";

export type ApiRequestOptions = {
  method?: HttpMethod;
  body?: unknown;
  signal?: AbortSignal;
  requestId?: string;
  idempotencyKey?: string;
  headers?: HeadersInit;
};

export type JsonResult = {
  kind: "json";
  status: number;
  requestId: string | null;
  data: unknown;
};

export type ContentResult = {
  kind: "content";
  status: number;
  requestId: string | null;
  mediaType: string;
  body: ArrayBuffer;
  contentLength: number | null;
  etag: string | null;
};

export type ApiClient = {
  requestJson(path: string, options?: ApiRequestOptions): Promise<JsonResult>;
  requestContent(path: string, options?: ApiRequestOptions): Promise<ContentResult>;
};

type FetchLike = (input: RequestInfo | URL, init?: RequestInit) => Promise<Response>;

export type ApiClientOptions = {
  baseUrl?: string;
  fetchImpl?: FetchLike;
};

function configuredBaseUrl(): string {
  const env: unknown = import.meta.env;
  if (typeof env !== "object" || env === null || !("VITE_API_BASE_URL" in env)) {
    return "";
  }
  const value = env.VITE_API_BASE_URL;
  return typeof value === "string" ? value : "";
}

function joinUrl(baseUrl: string, path: string): string {
  if (baseUrl.length === 0) {
    return path;
  }
  const base = baseUrl.endsWith("/") ? baseUrl.slice(0, -1) : baseUrl;
  const suffix = path.startsWith("/") ? path : `/${path}`;
  return `${base}${suffix}`;
}

function headerValue(headers: Headers, name: string): string | null {
  const value = headers.get(name);
  if (value === null || value.trim() === "") {
    return null;
  }
  return value;
}

function mediaTypeOf(contentType: string | null): string | null {
  if (contentType === null) {
    return null;
  }
  const [mediaType] = contentType.split(";");
  if (mediaType === undefined) {
    return null;
  }
  const trimmed = mediaType.trim().toLowerCase();
  return trimmed.length === 0 ? null : trimmed;
}

function contentLengthOf(raw: string | null): number | null {
  if (raw === null) {
    return null;
  }
  if (!/^\d+$/.test(raw)) {
    return null;
  }
  const parsed = Number(raw);
  return Number.isSafeInteger(parsed) ? parsed : null;
}

async function readProblem(
  response: Response,
  method: string,
  requestId: string | null,
): Promise<ApiProblemError> {
  const text = await response.text();
  let payload: unknown;
  try {
    payload = text.length === 0 ? null : JSON.parse(text);
  } catch {
    return new ApiProblemError(unexpectedProblem(response.status), { method, requestId });
  }
  const problem = parseProblemDetail(payload);
  if (problem === null) {
    return new ApiProblemError(unexpectedProblem(response.status), { method, requestId });
  }
  return new ApiProblemError({ ...problem, status: response.status }, { method, requestId });
}

export function createApiClient(options: ApiClientOptions = {}): ApiClient {
  const baseUrl = options.baseUrl ?? configuredBaseUrl();
  const fetchImpl = options.fetchImpl ?? ((input, init) => globalThis.fetch(input, init));

  async function perform(
    path: string,
    request: ApiRequestOptions,
    responseType: "json" | "content",
  ): Promise<JsonResult | ContentResult> {
    const method = request.method ?? "GET";
    const headers = new Headers(request.headers);
    if (request.requestId !== undefined && REQUEST_ID_PATTERN.test(request.requestId)) {
      headers.set(REQUEST_ID_HEADER, request.requestId);
    }
    if (
      request.idempotencyKey !== undefined &&
      REQUEST_ID_PATTERN.test(request.idempotencyKey)
    ) {
      headers.set(IDEMPOTENCY_KEY_HEADER, request.idempotencyKey);
    }
    let body: string | undefined;
    if (request.body !== undefined) {
      headers.set("Content-Type", "application/json");
      body = JSON.stringify(request.body);
    }

    let response: Response;
    try {
      response = await fetchImpl(joinUrl(baseUrl, path), {
        method,
        headers,
        body,
        signal: request.signal,
      });
    } catch (error) {
      if (isAbortError(error)) {
        throw error;
      }
      const sentRequestId = headerValue(headers, REQUEST_ID_HEADER);
      throw new ApiTransportError("The API could not be reached.", {
        method,
        requestId: sentRequestId,
        cause: error,
      });
    }

    const requestId = headerValue(response.headers, REQUEST_ID_HEADER);
    const mediaType = mediaTypeOf(response.headers.get("Content-Type"));
    if (!response.ok || mediaType === "application/problem+json") {
      throw await readProblem(response, method, requestId);
    }

    if (responseType === "content") {
      const buffer = await response.arrayBuffer();
      return {
        kind: "content",
        status: response.status,
        requestId,
        mediaType: mediaType ?? "application/octet-stream",
        body: buffer,
        contentLength: contentLengthOf(response.headers.get("Content-Length")),
        etag: headerValue(response.headers, "ETag"),
      };
    }

    const text = await response.text();
    let data: unknown;
    if (text.length === 0) {
      data = null;
    } else {
      try {
        data = JSON.parse(text);
      } catch {
        throw new ApiTransportError("The API returned a response that was not valid JSON.", {
          method,
          requestId,
        });
      }
    }
    return { kind: "json", status: response.status, requestId, data };
  }

  return {
    requestJson(path, request = {}) {
      return perform(path, request, "json") as Promise<JsonResult>;
    },
    requestContent(path, request = {}) {
      return perform(path, request, "content") as Promise<ContentResult>;
    },
  };
}
