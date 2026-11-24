import { execFileSync } from "node:child_process";
import { mkdtempSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const packageRoot = join(dirname(fileURLToPath(import.meta.url)), "..");
const committedPath = join(packageRoot, "src", "generated.ts");
const tempDir = mkdtempSync(join(tmpdir(), "rcp-contracts-"));
const generatedPath = join(tempDir, "generated.ts");
const binary = join(packageRoot, "node_modules", ".bin", "openapi-typescript");

try {
  execFileSync(binary, ["openapi.json", "-o", generatedPath], {
    cwd: packageRoot,
    stdio: ["ignore", "pipe", "pipe"],
  });
  const committed = readFileSync(committedPath, "utf8");
  const generated = readFileSync(generatedPath, "utf8");
  if (committed !== generated) {
    process.stderr.write("packages/contracts/src/generated.ts is stale\n");
    process.exitCode = 1;
  }
} finally {
  rmSync(tempDir, { recursive: true, force: true });
}
