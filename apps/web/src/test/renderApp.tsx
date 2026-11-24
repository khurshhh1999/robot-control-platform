import { QueryClientProvider } from "@tanstack/react-query";
import { render, type RenderResult } from "@testing-library/react";
import { createMemoryRouter, RouterProvider } from "react-router";

import type { ApiClient } from "../api/client";
import { ApiClientProvider } from "../api/context";
import { createQueryClient } from "../app/queryClient";
import { appRoutes } from "../app/router";

export function renderApp(path: string, client: ApiClient): RenderResult {
  const queryClient = createQueryClient({ retryDelay: 0 });
  const router = createMemoryRouter(appRoutes, { initialEntries: [path] });
  return render(
    <ApiClientProvider client={client}>
      <QueryClientProvider client={queryClient}>
        <RouterProvider router={router} />
      </QueryClientProvider>
    </ApiClientProvider>,
  );
}
