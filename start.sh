#!/usr/bin/env sh
set -e

cd "$(dirname "$0")"

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  echo "Node.js was not found. Install Node.js 22 first: https://nodejs.org/"
  exit 1
fi

if [ ! -d node_modules ]; then
  echo "First run: installing dependencies..."
  npm ci
fi

echo "Starting Matrix Coordinate Lab..."
npm start
