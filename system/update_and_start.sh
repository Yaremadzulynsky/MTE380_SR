#!/usr/bin/env bash
set -euo pipefail

# Repo root is one level above this script.
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SYSTEM_DIR="${REPO_ROOT}/system"

TARGET_BRANCH="${TARGET_BRANCH:-zain/perception}"
BASE_BRANCH="${BASE_BRANCH:-main}"

echo "[update_and_start] repo=${REPO_ROOT}"
echo "[update_and_start] base_branch=${BASE_BRANCH} target_branch=${TARGET_BRANCH}"

cd "${REPO_ROOT}"

git checkout "${BASE_BRANCH}"
git fetch origin
git checkout "${TARGET_BRANCH}"
git pull --ff-only

cd "${SYSTEM_DIR}"
docker compose up -d

echo "[update_and_start] done"
