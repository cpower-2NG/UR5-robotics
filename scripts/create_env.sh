#!/usr/bin/env bash
set -euo pipefail

# 创建或更新 Conda 环境（同名为 yolo_ros）
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${HERE%/scripts}"

echo "Using environment file: ${ROOT}/environment.yml"
if ! command -v conda >/dev/null 2>&1; then
  echo "conda not found. Install Miniconda or Anaconda first." >&2
  exit 2
fi

conda env create -f "${ROOT}/environment.yml" || conda env update -f "${ROOT}/environment.yml"

echo "Created/updated conda env: yolo_ros"
echo "Activate with: conda activate yolo_ros"
