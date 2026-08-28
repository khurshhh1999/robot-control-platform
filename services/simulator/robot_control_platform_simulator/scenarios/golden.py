"""Public-neutral golden seeds for scenario-generator regression checks."""

from __future__ import annotations

from typing import Final

GOLDEN_SEEDS: Final[tuple[int, ...]] = (1, 7, 13)

# Public-neutral hashes of the default generator and scene contract.
GOLDEN_CHECKSUMS: Final[dict[int, str]] = {
    1: "50d515e2a244736bbc43f791a2343135f1c4d7772d8a28a19d55853129aa9cf4",
    7: "3439403dd707eb84cf383408056f0c22f1f350330c27ba3fa1d1b61d5859df9c",
    13: "09017a5b82c7f9e2e979f92407f0a7cdd557b6f9cfb5b2683532075e589c174c",
}
