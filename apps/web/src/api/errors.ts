export type ProblemDetail = {
  type: string;
  title: string;
  status: number;
  detail: string;
  code: string;
  instance?: string;
};

type ProblemOptions = {
  method: string;
  requestId: string | null;
};

type TransportOptions = {
  method: string;
  requestId: string | null;
  cause?: unknown;
};

export class ApiProblemError extends Error {
  readonly status: number;
  readonly code: string;
  readonly title: string;
  readonly type: string;
  readonly instance: string | null;
  readonly requestId: string | null;
  readonly method: string;

  constructor(problem: ProblemDetail, options: ProblemOptions) {
    super(problem.detail);
    this.name = "ApiProblemError";
    this.status = problem.status;
    this.code = problem.code;
    this.title = problem.title;
    this.type = problem.type;
    this.instance = problem.instance ?? null;
    this.requestId = options.requestId;
    this.method = options.method;
  }
}

export class ApiTransportError extends Error {
  readonly method: string;
  readonly requestId: string | null;

  constructor(message: string, options: TransportOptions) {
    super(message, options.cause === undefined ? undefined : { cause: options.cause });
    this.name = "ApiTransportError";
    this.method = options.method;
    this.requestId = options.requestId;
  }
}

export function isAbortError(error: unknown): boolean {
  if (typeof error !== "object" || error === null || !("name" in error)) {
    return false;
  }
  return error.name === "AbortError";
}
