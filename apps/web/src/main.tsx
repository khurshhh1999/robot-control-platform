import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { QueryClientProvider } from "@tanstack/react-query";
import { RouterProvider } from "react-router";

import { createApiClient } from "./api/client";
import { ApiClientProvider } from "./api/context";
import { createQueryClient } from "./app/queryClient";
import { router } from "./app/router";
import "./styles/tokens.css";
import "./styles/global.css";

const rootElement = document.getElementById("root");
if (rootElement === null) {
  throw new Error("Root element is missing");
}

const queryClient = createQueryClient();
const apiClient = createApiClient();

createRoot(rootElement).render(
  <StrictMode>
    <ApiClientProvider client={apiClient}>
      <QueryClientProvider client={queryClient}>
        <RouterProvider router={router} />
      </QueryClientProvider>
    </ApiClientProvider>
  </StrictMode>,
);
