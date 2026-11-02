#!/bin/bash
# Installs the "anatomy" skill (isometric SVG figures) into ~/.claude/skills
# for Claude Code cloud sessions, pinned to a reviewed commit.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

# To upgrade: review the new upstream commit, then change this hash.
ANATOMY_REPO="https://github.com/wheresryan22/anatomy.git"
ANATOMY_COMMIT="1482e8469b0a9e0d8cc18ea2740a01423d29ae80"

dest="$HOME/.claude/skills/anatomy"
marker="$dest/.installed-commit"

if [ "$(cat "$marker" 2>/dev/null || true)" != "$ANATOMY_COMMIT" ]; then
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  git init --quiet "$tmp/anatomy"
  git -C "$tmp/anatomy" fetch --quiet --depth 1 "$ANATOMY_REPO" "$ANATOMY_COMMIT"
  git -C "$tmp/anatomy" checkout --quiet FETCH_HEAD
  rm -rf "$tmp/anatomy/.git"
  echo "$ANATOMY_COMMIT" > "$tmp/anatomy/.installed-commit"
  mkdir -p "$(dirname "$dest")"
  rm -rf "$dest"
  mv "$tmp/anatomy" "$dest"
fi
