#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${JOYCON_VENV_DIR:-$ROOT_DIR/.venv}"
SKIP_SYSTEM_INSTALL="${JOYCON_SKIP_SYSTEM_INSTALL:-0}"
SKIP_DRIVER_INSTALL="${JOYCON_SKIP_DRIVER_INSTALL:-0}"

log() {
  printf '\033[1;32m==>\033[0m %s\n' "$*"
}

warn() {
  printf '\033[1;33mwarning:\033[0m %s\n' "$*" >&2
}

die() {
  printf '\033[1;31merror:\033[0m %s\n' "$*" >&2
  exit 1
}

run_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

require_ubuntu() {
  if [[ ! -r /etc/os-release ]]; then
    warn "Cannot read /etc/os-release; continuing anyway."
    return
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  log "host: ${PRETTY_NAME:-unknown}, kernel: $(uname -r), arch: $(uname -m)"

  if [[ "${ID:-}" != "ubuntu" ]]; then
    warn "This script is tuned for Ubuntu 24.04; current ID=${ID:-unknown}."
  elif [[ "${VERSION_ID:-}" != "24.04" ]]; then
    warn "This script is tuned for Ubuntu 24.04; current VERSION_ID=${VERSION_ID:-unknown}."
  fi
}

install_system_packages() {
  if [[ "$SKIP_SYSTEM_INSTALL" == "1" ]]; then
    log "skipping apt packages because JOYCON_SKIP_SYSTEM_INSTALL=1"
    return
  fi

  log "installing Ubuntu system packages"
  run_sudo apt-get update
  run_sudo apt-get install -y \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    dkms \
    git \
    libevdev-dev \
    libhidapi-dev \
    libhidapi-hidraw0 \
    libhidapi-libusb0 \
    libudev-dev \
    linux-headers-"$(uname -r)" \
    pkg-config \
    python3 \
    python3-dev \
    python3-venv \
    rfkill \
    bluez \
    bluetooth
}

install_uv() {
  export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

  if command -v uv >/dev/null 2>&1; then
    log "uv already installed: $(uv --version)"
    return
  fi

  command -v curl >/dev/null 2>&1 || die "curl is required to install uv."
  log "installing uv"
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
  command -v uv >/dev/null 2>&1 || die "uv was not found after installation."
}

install_python_env() {
  log "creating uv virtual environment: $VENV_DIR"
  uv venv --python python3 "$VENV_DIR"

  log "installing Python dependencies with uv"
  uv pip install --python "$VENV_DIR/bin/python" --upgrade \
    ansitable \
    hidapi \
    ipykernel \
    matplotlib \
    numpy \
    progress \
    pyglm \
    scipy \
    typing_extensions

  uv pip install --python "$VENV_DIR/bin/python" --no-deps -e "$ROOT_DIR"

  log "verifying hidapi"
  "$VENV_DIR/bin/python" - <<'PY'
import hid
print("hid module:", getattr(hid, "__file__", "<unknown>"))
print("hidapi enumerate OK, devices visible:", len(hid.enumerate(0, 0)))
PY
}

patch_hid_nintendo_for_new_kernels() {
  local src="$ROOT_DIR/joyconrobotics/system_lib/dkms-hid-nintendo/src/hid-nintendo.c"
  [[ -f "$src" ]] || return

  if grep -q "#include <linux/unaligned.h>" "$src"; then
    log "hid-nintendo unaligned include is already compatible"
    return
  fi

  if grep -q "#include <asm/unaligned.h>" "$src"; then
    log "patching hid-nintendo unaligned include for newer Ubuntu kernels"
    sed -i 's/#include <asm\/unaligned.h>/#include <linux\/unaligned.h>/' "$src"
  fi
}

install_kernel_and_joycond() {
  if [[ "$SKIP_DRIVER_INSTALL" == "1" ]]; then
    log "skipping DKMS/joycond install because JOYCON_SKIP_DRIVER_INSTALL=1"
    return
  fi

  log "installing hid-nintendo DKMS module, joycond, hidapi packages, and udev rules"
  make -C "$ROOT_DIR" install
}

configure_bluetooth() {
  if [[ "$SKIP_SYSTEM_INSTALL" == "1" ]]; then
    return
  fi

  log "enabling Bluetooth service"
  run_sudo systemctl enable --now bluetooth
  if command -v rfkill >/dev/null 2>&1; then
    run_sudo rfkill unblock bluetooth || true
  fi
}

reload_udev_rules() {
  log "installing udev rules for Joy-Con and Nintendo Switch Pro Controller"
  run_sudo cp "$ROOT_DIR/udev/99-nitendo.rules" /etc/udev/rules.d/99-nitendo.rules
  run_sudo udevadm control --reload-rules
  run_sudo udevadm trigger || true
}

main() {
  require_ubuntu
  log "repo: $ROOT_DIR"
  log "venv: $VENV_DIR"

  install_system_packages
  install_uv
  install_python_env
  patch_hid_nintendo_for_new_kernels
  install_kernel_and_joycond
  reload_udev_rules
  configure_bluetooth

  cat <<EOF

Setup complete.

Pair/connect the controller:
  Joy-Con: hold SYNC until LEDs scroll, pair in Bluetooth settings, then press L/R binding buttons.
  Pro Controller: hold SYNC until LEDs scroll, pair in Bluetooth settings or connect USB, then press both triggers if joycond asks for pairing.

Test commands:
  source "$VENV_DIR/bin/activate"
  python "$ROOT_DIR/script/ubuntu_nintendo_controller_test.py" --list
  python "$ROOT_DIR/script/ubuntu_nintendo_controller_test.py" --controller pro
  python "$ROOT_DIR/script/ubuntu_nintendo_controller_test.py" --controller right

Options:
  JOYCON_VENV_DIR=/path/to/venv changes the uv venv path.
  JOYCON_SKIP_DRIVER_INSTALL=1 skips DKMS/joycond installation.
  JOYCON_SKIP_SYSTEM_INSTALL=1 skips apt/Bluetooth service steps.
EOF
}

main "$@"
