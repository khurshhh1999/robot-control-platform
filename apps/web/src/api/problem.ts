import { z } from "zod";

import type { ProblemDetail } from "./errors";

export const problemDetailSchema = z.strictObject({
  type: z.string().min(1),
  title: z.string().min(1),
  status: z.number().int(),
  detail: z.string(),
  code: z.string().min(1),
  instance: z.string().min(1).optional(),
});

export function parseProblemDetail(payload: unknown): ProblemDetail | null {
  const parsed = problemDetailSchema.safeParse(payload);
  if (!parsed.success) {
    return null;
  }
  return parsed.data;
}

export function unexpectedProblem(status: number): ProblemDetail {
  return {
    type: "urn:robot-control-platform:problem:INTERNAL_ERROR",
    title: "Internal Error",
    status,
    detail: "The API returned an unexpected error response.",
    code: "INTERNAL_ERROR",
  };
}
