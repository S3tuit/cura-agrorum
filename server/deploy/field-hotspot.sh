#!/usr/bin/env bash
set -euo pipefail

# Fedora/NetworkManager helper for field collection.
# Starts a 2.4 GHz laptop hotspot with a stable gateway address for ESP32 nodes.

CON_NAME="${CURA_HOTSPOT_CON_NAME:-cura-field-hotspot}"
SSID="${CURA_HOTSPOT_SSID:-cura-field}"
PASSWORD="${CURA_HOTSPOT_PASSWORD:-cura-field-password}"
ADDRESS_CIDR="${CURA_HOTSPOT_ADDRESS_CIDR:-10.42.0.1/24}"
CHANNEL="${CURA_HOTSPOT_CHANNEL:-6}"
FIREWALL_ZONE="${CURA_HOTSPOT_FIREWALL_ZONE:-trusted}"
GATEWAY_PORT="${CURA_GATEWAY_PORT:-18032}"

usage() {
  cat <<EOF
Usage: sudo $0 [up|down|status]

Environment overrides:
  CURA_WIFI_IFACE             Wi-Fi interface to use, for example wlp2s0
  CURA_HOTSPOT_CON_NAME       NetworkManager connection name [$CON_NAME]
  CURA_HOTSPOT_SSID           Hotspot SSID [$SSID]
  CURA_HOTSPOT_PASSWORD       WPA-PSK password [$PASSWORD]
  CURA_HOTSPOT_ADDRESS_CIDR   Laptop AP address [$ADDRESS_CIDR]
  CURA_HOTSPOT_CHANNEL        2.4 GHz channel [$CHANNEL]
  CURA_HOTSPOT_FIREWALL_ZONE  firewalld zone for hotspot interface [$FIREWALL_ZONE]
  CURA_GATEWAY_PORT           Cura server TCP port [$GATEWAY_PORT]

The ESP32 firmware should connect to SSID "$SSID" and send to ${ADDRESS_CIDR%/*}:$GATEWAY_PORT.
EOF
}

need_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo so NetworkManager can create and activate the hotspot." >&2
    exit 1
  fi
}

need_nmcli() {
  if ! command -v nmcli >/dev/null 2>&1; then
    echo "nmcli not found. On Fedora, install NetworkManager Wi-Fi support:" >&2
    echo "  sudo dnf install NetworkManager NetworkManager-wifi" >&2
    exit 1
  fi
}

wifi_iface() {
  if [[ -n "${CURA_WIFI_IFACE:-}" ]]; then
    printf '%s\n' "$CURA_WIFI_IFACE"
    return
  fi

  nmcli -t -f DEVICE,TYPE device status |
    awk -F: '$2 == "wifi" { print $1; exit }'
}

ensure_connection() {
  local iface="$1"

  nmcli radio wifi on

  if nmcli -t -f NAME connection show | grep -Fxq "$CON_NAME"; then
    nmcli connection modify "$CON_NAME" \
      connection.interface-name "$iface" \
      connection.autoconnect no \
      802-11-wireless.mode ap \
      802-11-wireless.ssid "$SSID" \
      802-11-wireless.band bg \
      802-11-wireless.channel "$CHANNEL" \
      802-11-wireless-security.key-mgmt wpa-psk \
      802-11-wireless-security.psk "$PASSWORD" \
      ipv4.method shared \
      ipv4.addresses "$ADDRESS_CIDR" \
      ipv6.method ignore \
      connection.zone "$FIREWALL_ZONE"
  else
    nmcli connection add type wifi ifname "$iface" con-name "$CON_NAME" ssid "$SSID"
    nmcli connection modify "$CON_NAME" \
      connection.autoconnect no \
      802-11-wireless.mode ap \
      802-11-wireless.band bg \
      802-11-wireless.channel "$CHANNEL" \
      802-11-wireless-security.key-mgmt wpa-psk \
      802-11-wireless-security.psk "$PASSWORD" \
      ipv4.method shared \
      ipv4.addresses "$ADDRESS_CIDR" \
      ipv6.method ignore \
      connection.zone "$FIREWALL_ZONE"
  fi
}

open_gateway_firewall() {
  if ! command -v firewall-cmd >/dev/null 2>&1; then
    echo "firewall-cmd not found; if TCP connects time out, install firewalld or open $GATEWAY_PORT/tcp manually." >&2
    return
  fi
  if ! firewall-cmd --state >/dev/null 2>&1; then
    echo "firewalld is not running; skipping firewall rule." >&2
    return
  fi

  firewall-cmd --zone="$FIREWALL_ZONE" --add-port="$GATEWAY_PORT/tcp" >/dev/null
}

hotspot_up() {
  need_root
  need_nmcli

  local iface
  iface="$(wifi_iface)"
  if [[ -z "$iface" ]]; then
    echo "No Wi-Fi interface found. Set CURA_WIFI_IFACE explicitly." >&2
    exit 1
  fi

  ensure_connection "$iface"
  nmcli connection up "$CON_NAME"
  open_gateway_firewall

  cat <<EOF
Hotspot is up.
  interface: $iface
  ssid:      $SSID
  password:  $PASSWORD
  address:   $ADDRESS_CIDR
  firewall:  $FIREWALL_ZONE allows $GATEWAY_PORT/tcp
  gateway:   ${ADDRESS_CIDR%/*}
  server:    ${ADDRESS_CIDR%/*}:$GATEWAY_PORT
EOF
}

hotspot_down() {
  need_root
  need_nmcli
  nmcli connection down "$CON_NAME"
}

hotspot_status() {
  need_nmcli
  nmcli connection show "$CON_NAME"
}

case "${1:-up}" in
  up)
    hotspot_up
    ;;
  down)
    hotspot_down
    ;;
  status)
    hotspot_status
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
