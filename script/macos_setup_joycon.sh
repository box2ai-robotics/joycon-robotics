#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${JOYCON_VENV_DIR:-$ROOT_DIR/.venv-macos}"

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "This setup script is for macOS. Current OS: $(uname -s)"
  exit 1
fi

echo "==> macOS: $(sw_vers -productVersion), arch: $(uname -m)"
echo "==> repo: $ROOT_DIR"
echo "==> venv: $VENV_DIR"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 was not found. Install Python 3 first, then rerun this script."
  exit 1
fi

if command -v brew >/dev/null 2>&1; then
  BREW_PREFIX="$(brew --prefix)"
  export PATH="$BREW_PREFIX/bin:$PATH"

  if ! command -v uv >/dev/null 2>&1; then
    echo "==> installing uv with Homebrew"
    brew install uv
  else
    echo "==> uv already installed"
  fi

  if ! brew list hidapi >/dev/null 2>&1; then
    echo "==> installing native hidapi with Homebrew"
    brew install hidapi pkg-config
  else
    echo "==> Homebrew hidapi already installed"
  fi

  if brew list pkg-config >/dev/null 2>&1; then
    :
  else
    echo "==> installing pkg-config with Homebrew"
    brew install pkg-config
  fi

  export PKG_CONFIG_PATH="$BREW_PREFIX/opt/hidapi/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
else
  echo "Homebrew was not found."
  echo "Install it from https://brew.sh, then rerun this script."
  echo "This script uses uv for Python dependency management and Homebrew for native hidapi."
  exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
  echo "uv was not found after setup."
  exit 1
fi

echo "==> creating virtual environment"
uv venv --python python3 "$VENV_DIR"
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

echo "==> installing Python runtime dependencies with uv"
uv pip install --python "$VENV_DIR/bin/python" --upgrade hidapi numpy

echo "==> verifying hidapi import"
"$VENV_DIR/bin/python" - <<'PY'
import hid
print("hid module:", getattr(hid, "__file__", "<unknown>"))
print("hidapi enumerate OK, devices visible:", len(hid.enumerate(0, 0)))
PY

cat <<EOF

Setup complete.

Next steps:
  1. Put the Joy-Con or Pro Controller into pairing mode with the SYNC button.
  2. Pair it in macOS System Settings > Bluetooth.
  3. Keep it connected, then run:

     source "$VENV_DIR/bin/activate"
     python "$ROOT_DIR/script/macos_joycon_function_test.py" --side right

For a left Joy-Con, use:
     python "$ROOT_DIR/script/macos_joycon_function_test.py" --side left

For a Nintendo Switch Pro Controller, use:
     python "$ROOT_DIR/script/macos_joycon_function_test.py" --controller pro
EOF
