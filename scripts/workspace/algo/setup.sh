#!/bin/bash
set -e
cd "$WORKSPACE"
rm -rf "$REPO_NAME"
cp -a "$REPO_ROOT" "$REPO_NAME"
grep -qxF "build/" "$REPO_NAME/.gitignore" 2>/dev/null || echo "build/" >> "$REPO_NAME/.gitignore"
