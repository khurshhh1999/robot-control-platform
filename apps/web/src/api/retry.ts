import { ApiProblemError, ApiTransportError, isAbortError } from "./errors";

/** Extra attempts after the first failure for a transient read. */
export const MAX_TRANSIENT_READ_RETRIES = 2;

const READ_METHODS = new Set(["GET", "HEAD"]);

export function isReadMethod(method: string): boolean {
  return READ_METHODS.has(method.toUpperCase());
}

export function isTransientReadFailure(error: unknown): boolean {
  if (isAbortError(error)) {
    return false;
  }
  if (error instanceof ApiProblemError) {
    if (!isReadMethod(error.method)) {
      return false;
    }
    if (error.status >= 400 && error.status < 500) {
      return false;
    }
    return error.status >= 500;
  }
  if (error instanceof ApiTransportError) {
    return isReadMethod(error.method);
  }
  return false;
}

/**
 * TanStack Query calls this with the number of failures already observed,
 * starting at 0. Returning true schedules one more read attempt.
 */
export function shouldRetryRead(failureCount: number, error: unknown): boolean {
  if (failureCount >= MAX_TRANSIENT_READ_RETRIES) {
    return false;
  }
  return isTransientReadFailure(error);
}
