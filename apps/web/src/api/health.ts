import type { paths } from "@robot-control-platform/contracts";

import { z } from "zod";

import type { ApiClient } from "./client";

export const LIVE_HEALTH_PATH = "/health/live" satisfies keyof paths;

const liveHealthSchema = z.strictObject({
  status: z.string(),
});

export async function readLiveHealth(
  client: ApiClient,
  signal: AbortSignal,
): Promise<{ status: string }> {
  const result = await client.requestJson(LIVE_HEALTH_PATH, { method: "GET", signal });
  const parsed = liveHealthSchema.safeParse(result.data);
  if (!parsed.success) {
    return { status: "" };
  }
  return { status: parsed.data.status };
}
