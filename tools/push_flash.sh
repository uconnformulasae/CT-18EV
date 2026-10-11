#!/bin/sh
# Push all flash snapshots to GitHub.

cd "$(dirname "$0")/.." || exit 1
git push origin 'refs/flash/*:refs/flash/*'
