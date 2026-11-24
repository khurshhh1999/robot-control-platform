import axe from "axe-core";
import { expect } from "vitest";

export async function expectNoAxeViolations(container: HTMLElement): Promise<void> {
  // jsdom has no canvas, so the color-contrast rule cannot measure pixels here.
  const results = await axe.run(container, {
    rules: {
      "color-contrast": { enabled: false },
    },
  });
  const summary = results.violations.map((violation) => {
    const target = violation.nodes[0]?.target.join(" ") ?? "";
    return `${violation.id}: ${violation.help} ${target}`.trim();
  });
  expect(summary).toEqual([]);
}
