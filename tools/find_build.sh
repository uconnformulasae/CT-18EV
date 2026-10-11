#!/bin/sh
# Look up the code the car ran from the AiM Version Hi/Lo channels.
#   tools/find_build.sh <version_hi> <version_lo>

if [ $# -ne 2 ]; then
    echo "usage: $0 <version_hi> <version_lo>" >&2
    exit 1
fi

cd "$(dirname "$0")/.." || exit 1

short=$(printf '%04x%04x' "$1" "$2") || exit 1
if [ "$short" = "00000000" ]; then
    echo "Hash 0: build had no version info"
    exit 1
fi

git fetch --quiet origin 'refs/flash/*:refs/flash/*' 2>/dev/null

full=$(git rev-parse --verify --quiet "$short^{commit}")
if [ -z "$full" ]; then
    echo "$short: not found (try git fetch, or the snapshot was never pushed)"
    exit 1
fi

url=$(git remote get-url origin 2>/dev/null |
    sed -e 's#^git@github.com:#https://github.com/#' -e 's#\.git$##')

git log -1 --format='%H%n%ad  %an%n%s' "$full"
if [ -n "$(git for-each-ref --points-at "$full" refs/flash/)" ]; then
    echo
    echo "Snapshot of uncommitted code on top of $(git rev-parse --short "$full^"):"
    git diff --stat "$full^" "$full"
fi
[ -n "$url" ] && echo && echo "$url/commit/$full"
exit 0
