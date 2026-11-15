#!/usr/bin/env bash
set -e
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/view_landing_camera.sh" "$@"
