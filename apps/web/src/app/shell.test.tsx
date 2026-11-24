import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { describe, expect, it, vi } from "vitest";

import type { ApiClient, ContentResult, JsonResult } from "../api/client";
import { ApiTransportError } from "../api/errors";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { expectNoAxeViolations } from "../test/axe";
import { renderApp } from "../test/renderApp";

function jsonResult(data: unknown): JsonResult {
  return { kind: "json", status: 200, requestId: "req-shell", data };
}

function jsonClient(data: unknown): ApiClient {
  return {
    requestJson: vi.fn(async () => jsonResult(data)),
    requestContent: vi.fn(async (): Promise<ContentResult> => {
      throw new Error("content was not requested");
    }),
  };
}

function unreachableClient(): ApiClient {
  return {
    requestJson: vi.fn(async () => {
      throw new ApiTransportError("The API could not be reached.", {
        method: "GET",
        requestId: null,
      });
    }),
    requestContent: vi.fn(async (): Promise<ContentResult> => {
      throw new Error("content was not requested");
    }),
  };
}

describe("review shell", () => {
  it("renders the skip link, navigation, and a reachable API", async () => {
    const user = userEvent.setup();
    const view = renderApp("/", jsonClient({ status: "ok" }));
    expect(await screen.findByText("The API is reachable.")).toBeInTheDocument();
    expect(screen.getByRole("navigation", { name: "Primary" })).toBeInTheDocument();
    await user.tab();
    expect(screen.getByRole("link", { name: "Skip to content" })).toHaveFocus();
    await user.keyboard("{Enter}");
    expect(screen.getByRole("main")).toHaveFocus();
    await expectNoAxeViolations(view.container);
  });

  it("shows an empty state when the API status is unusable", async () => {
    const view = renderApp("/", jsonClient({ status: "" }));
    expect(await screen.findByText("The API did not report a usable status.")).toBeInTheDocument();
    await expectNoAxeViolations(view.container);
  });

  it("shows a loading state while the API probe is pending", async () => {
    const client: ApiClient = {
      requestJson: vi.fn(() => new Promise<JsonResult>(() => undefined)),
      requestContent: vi.fn(async (): Promise<ContentResult> => {
        throw new Error("content was not requested");
      }),
    };
    const view = renderApp("/", client);
    expect(screen.getByRole("status")).toHaveTextContent("Checking whether the API is reachable.");
    await expectNoAxeViolations(view.container);
  });

  it("shows the API-down state and retries when asked", async () => {
    const user = userEvent.setup();
    const client = unreachableClient();
    const view = renderApp("/", client);
    expect(await screen.findByRole("heading", { name: "API unavailable" })).toBeInTheDocument();
    const callsBeforeRetry = vi.mocked(client.requestJson).mock.calls.length;
    await user.click(screen.getByRole("button", { name: "Try again" }));
    expect(await screen.findByRole("heading", { name: "API unavailable" })).toBeInTheDocument();
    expect(vi.mocked(client.requestJson).mock.calls.length).toBeGreaterThan(callsBeforeRetry);
    await expectNoAxeViolations(view.container);
  });

  it("renders the not-found page inside the shell", async () => {
    const user = userEvent.setup();
    const view = renderApp("/missing", jsonClient({ status: "ok" }));
    expect(screen.getByRole("heading", { name: "Page not found" })).toBeInTheDocument();
    expect(screen.getByRole("link", { name: "Skip to content" })).toBeInTheDocument();
    await expectNoAxeViolations(view.container);
    await user.click(screen.getByRole("link", { name: "Back to overview" }));
    expect(await screen.findByRole("heading", { name: "Overview" })).toBeInTheDocument();
  });

  it("catches render errors without showing the thrown detail", async () => {
    const user = userEvent.setup();
    const consoleError = vi.spyOn(console, "error").mockImplementation(() => undefined);
    let fail = true;
    function Boom() {
      if (fail) {
        throw new Error("hidden-detail /var/secret");
      }
      return <p>Recovered</p>;
    }
    try {
      const view = render(
        <ErrorBoundary>
          <Boom />
        </ErrorBoundary>,
      );
      expect(screen.getByRole("heading", { name: "Something went wrong" })).toBeInTheDocument();
      expect(screen.queryByText(/hidden-detail/)).not.toBeInTheDocument();
      await expectNoAxeViolations(view.container);
      fail = false;
      await user.click(screen.getByRole("button", { name: "Try again" }));
      expect(screen.getByText("Recovered")).toBeInTheDocument();
    } finally {
      consoleError.mockRestore();
    }
  });
});
