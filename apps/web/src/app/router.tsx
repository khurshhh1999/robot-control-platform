import { createBrowserRouter, type RouteObject } from "react-router";

import { NotFoundPage } from "../components/NotFoundPage";
import { RootLayout } from "../components/Shell";
import { OverviewPage } from "./OverviewPage";

export const appRoutes: RouteObject[] = [
  {
    path: "/",
    element: <RootLayout />,
    children: [
      { index: true, element: <OverviewPage /> },
      { path: "*", element: <NotFoundPage /> },
    ],
  },
];

export const router = createBrowserRouter(appRoutes);
