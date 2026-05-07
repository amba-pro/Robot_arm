#!/usr/bin/env bash
set -euo pipefail

echo "[ARM4] Installing OpenSSH server..."
sudo apt-get update
sudo apt-get install -y openssh-server

echo "[ARM4] Enabling SSH service..."
sudo systemctl enable ssh
sudo systemctl restart ssh

echo "[ARM4] Applying basic sshd hardening..."
SSHD_CONFIG="/etc/ssh/sshd_config"

sudo cp "${SSHD_CONFIG}" "${SSHD_CONFIG}.bak.arm4.$(date +%Y%m%d%H%M%S)"

sudo sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication yes/' "${SSHD_CONFIG}"
sudo sed -i 's/^#\?PubkeyAuthentication .*/PubkeyAuthentication yes/' "${SSHD_CONFIG}"
sudo sed -i 's/^#\?PermitRootLogin .*/PermitRootLogin no/' "${SSHD_CONFIG}"

if ! grep -q "^ClientAliveInterval" "${SSHD_CONFIG}"; then
  echo "ClientAliveInterval 300" | sudo tee -a "${SSHD_CONFIG}" >/dev/null
fi

if ! grep -q "^ClientAliveCountMax" "${SSHD_CONFIG}"; then
  echo "ClientAliveCountMax 2" | sudo tee -a "${SSHD_CONFIG}" >/dev/null
fi

echo "[ARM4] Validating sshd config..."
sudo sshd -t
sudo systemctl restart ssh

echo "[ARM4] SSH status:"
systemctl status ssh --no-pager -l | sed -n '1,12p'

echo "[ARM4] Done. Connect using: ssh <user>@<host_ip>"
