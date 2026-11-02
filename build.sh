#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
docker build --platform linux/amd64 -t corridor-solution:final .
