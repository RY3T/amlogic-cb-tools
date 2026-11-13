#!/usr/bin/env bash
# Build and publish an amlogic-cb-tools release.
#
#   scripts/release.sh ry3t-vX.Y.Z [NOTES_FILE]
#
# If the tag does not exist yet, it tags HEAD, which must be a clean `main`
# equal to origin/main. If it exists (pushed earlier), it releases that tag.
# Either way the binaries are built from the tag in a temporary worktree, for
# the Amlogic control boards (aarch64 musl), and published with SHA256SUMS as
# a GitHub release on RY3T/amlogic-cb-tools. The installer
# (RY3T/mujina-installer) downloads them from there.
set -euo pipefail
die() { echo "!! $*" >&2; exit 1; }

TAG=${1:?usage: scripts/release.sh ry3t-vX.Y.Z [NOTES_FILE]}
NOTES=${2:-}
[[ $TAG =~ ^ry3t-v[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "tag must look like ry3t-v1.2.3"
cd "$(git rev-parse --show-toplevel)"
REPO=RY3T/amlogic-cb-tools
TARGET=aarch64-unknown-linux-musl
[ -z "$NOTES" ] || [ -f "$NOTES" ] || die "$NOTES does not exist"
NOTES=${NOTES:+$(cd "$(dirname "$NOTES")" && pwd)/$(basename "$NOTES")}
gh release view "$TAG" -R "$REPO" >/dev/null 2>&1 && die "release $TAG already exists"

git fetch -q origin main --tags
if ! git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
  [ "$(git rev-parse --abbrev-ref HEAD)" = main ] || die "check out main first"
  [ "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)" ] || die "main is not origin/main: push or pull first"
  [ -z "$(git status --porcelain --untracked-files=no)" ] || die "uncommitted changes"
  git tag -a "$TAG" -m "$TAG"
  git push -q origin "$TAG"
fi
COMMIT=$(git rev-parse "$TAG^{commit}")

WT=$(mktemp -d)/cb-tools
DIST=$PWD/dist/$TAG
trap 'git worktree remove --force "$WT" >/dev/null 2>&1 || true' EXIT
git worktree add -q --detach "$WT" "$TAG"
( cd "$WT" && cargo build --release --locked --target "$TARGET" --bins \
    --config "target.$TARGET.linker=\"rust-lld\"" )

rm -rf "$DIST" && mkdir -p "$DIST"
for src in "$WT"/src/bin/*.rs; do
  b=$(basename "$src" .rs)
  cp "$WT/target/$TARGET/release/$b" "$DIST/$b"
done
( cd "$DIST" && shasum -a 256 * > SHA256SUMS && cat SHA256SUMS )

if [ -n "$NOTES" ]; then notes=(--notes-file "$NOTES")
else notes=(--notes "amlogic-cb-tools $TAG (commit $COMMIT), aarch64-unknown-linux-musl."); fi
gh release create "$TAG" -R "$REPO" --verify-tag --title "$TAG" "${notes[@]}" "$DIST"/*
echo ">> released $TAG ($COMMIT): https://github.com/$REPO/releases/tag/$TAG"
