#!/bin/bash
# =============================================================================
# STEELHAUS — Raspberry Pi Setup Script
# For: Raspberry Pi OS Bookworm 64-bit
# Run once on a fresh install: sudo bash setup.sh
# =============================================================================

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root: sudo bash setup.sh"
  exit 1
fi

ERRORS=""

echo ""
echo "=============================================="
echo "  STEELHAUS — Raspberry Pi Setup"
echo "  Raspberry Pi OS Bookworm 64-bit"
echo "=============================================="
echo ""

# ── 1. Update system packages ─────────────────────────────────────────────────
echo "[1/8] Updating system packages..."
apt-get update -qq
apt-get install -y \
  python3 python3-pip \
  sqlite3 \
  curl \
  chromium-browser \
  xinit \
  xorg \
  matchbox-window-manager \
  unclutter \
  x11-xserver-utils \
  2>/dev/null
# cage is NOT used — xinit/X11 is used exclusively on Pi 4 Bookworm

# Verify critical packages installed successfully
MISSING=""
for pkg in chromium-browser xinit matchbox-window-manager unclutter; do
  if ! dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
    MISSING="$MISSING $pkg"
  fi
done
if [ -n "$MISSING" ]; then
  echo "      WARNING: The following packages could not be installed:$MISSING"
  echo "      The kiosk display may not work. Check your internet connection and try again."
  ERRORS="$ERRORS\n  - Missing packages:$MISSING"
else
  echo "      All required packages installed successfully."
fi
echo "      Done."

# ── 2. Install Python packages ────────────────────────────────────────────────
echo "[2/8] Installing Python packages..."
pip3 install --break-system-packages --quiet flask flask-cors pyserial
if [ $? -ne 0 ]; then
  ERRORS="$ERRORS\n  - Python package install failed"
  echo "      WARNING: pip install had errors — trying again with --ignore-installed"
  pip3 install --break-system-packages --ignore-installed --quiet flask flask-cors pyserial
fi
echo "      Done."

# ── 3. Create project directories ─────────────────────────────────────────────
echo "[3/8] Creating project directories..."
mkdir -p /home/pi/kiln_controller/data
mkdir -p /home/pi/kiln_controller/static
chown -R pi:pi /home/pi/kiln_controller
echo "      Done."

# ── 4. Detect ESP32 serial port and create systemd service ───────────────────
echo "[4/8] Setting up system service..."

ESP_PORT=""
for port in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
  if [ -e "$port" ]; then
    ESP_PORT="$port"
    echo "      ESP32 found at: $ESP_PORT"
    break
  fi
done
if [ -z "$ESP_PORT" ]; then
  ESP_PORT="/dev/ttyUSB0"
  echo "      ESP32 not detected — defaulting to /dev/ttyUSB0"
fi

usermod -aG dialout pi

cat > /etc/systemd/system/kiln-controller.service << EOF
[Unit]
Description=STEELHAUS Heat Treat Oven Controller
After=network.target multi-user.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/kiln_controller
Environment=KILN_DB=/home/pi/kiln_controller/data/kiln.db
Environment=KILN_SERIAL=${ESP_PORT}
Environment=KILN_PORT=5000
ExecStart=/usr/bin/python3 app.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
KillSignal=SIGTERM
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable kiln-controller.service
echo "      Service created and enabled."

# ── 5. Configure kiosk display (force X11, disable Wayland) ──────────────────
echo "[5/8] Configuring kiosk display..."
#
# STEELHAUS requires X11 for reliable touchscreen calibration and cursor
# hiding. Wayland has compatibility issues with these features on Pi hardware.
# We force X11 via raspi-config regardless of what was previously configured.
#
# Boot flow after setup:
#   kernel -> TTY1 auto-login as pi -> .bash_profile -> start_kiosk.sh
#   -> xinit -> xinitrc.sh -> xrandr rotate -> touch fix -> Chromium fullscreen
#   SSH sessions are NOT affected (tty check guards .bash_profile).

# Force X11 display server
if command -v raspi-config >/dev/null 2>&1; then
  raspi-config nonint do_wayland W1 2>/dev/null && \
    echo "      Forced X11 display server (Wayland disabled)." || \
    echo "      WARNING: Could not set X11 via raspi-config — may already be X11."
else
  echo "      raspi-config not found — skipping display server switch."
fi

# cage_wrapper.sh is NOT used — cage/Wayland causes display rotation and
# touchscreen calibration failures on Pi 4 Bookworm. xinit/X11 is used instead.

# ── xinitrc.sh — X11 kiosk session ───────────────────────────────────────────
cat > /home/pi/kiln_controller/xinitrc.sh << 'XINIT'
#!/bin/bash
# STEELHAUS Kiosk — X11 xinit session
#
# Rotate display 90° CCW (portrait, connector at bottom).
# xrandr --rotate left works with vc4-kms-v3d on Pi 4 Bookworm X11.
# display_rotate= in config.txt is ignored by the KMS driver — do not use it.
xrandr --output HDMI-1 --rotate left 2>/dev/null || \
  xrandr --output HDMI-A-1 --rotate left 2>/dev/null || true

# Fix touch input to match rotated display.
# Works with any touchscreen brand — finds the first non-mouse, non-virtual
# slave pointer and applies the identity matrix. The udev rule handles the
# actual 90° CCW calibration at the driver level via libinput, so xinput
# must stay at identity (1 0 0 / 0 1 0 / 0 0 1) to avoid double-rotating.
TOUCH_ID=$(xinput list | grep -i "slave.*pointer" | \
           grep -iv "virtual\|vc4\|hdmi\|mouse" | \
           grep -o 'id=[0-9]*' | head -1 | cut -d= -f2)
if [ -n "$TOUCH_ID" ]; then
  xinput set-prop "$TOUCH_ID" "Coordinate Transformation Matrix" 1 0 0 0 1 0 0 0 1
fi

unclutter -idle 0.1 -root &
matchbox-window-manager -use_titlebar no &
exec chromium-browser \
  --kiosk \
  --noerrdialogs \
  --disable-infobars \
  --no-first-run \
  --disable-restore-session-state \
  --disable-session-crashed-bubble \
  --disable-translate \
  --disable-features=TranslateUI \
  --hide-scrollbars \
  --touch-events=enabled \
  --password-store=basic \
  --check-for-update-interval=31536000 \
  http://localhost:5000
XINIT
chmod +x /home/pi/kiln_controller/xinitrc.sh
chown pi:pi /home/pi/kiln_controller/xinitrc.sh

# ── start_kiosk.sh — main kiosk launcher ──────────────────────────────────────
cat > /home/pi/kiln_controller/start_kiosk.sh << 'KIOSK'
#!/bin/bash
# Wait for Flask to be ready (up to 60s)
for i in $(seq 1 30); do
  if curl -s http://localhost:5000 > /dev/null 2>&1; then
    break
  fi
  sleep 2
done

# Always use xinit/X11 — cage/Wayland has known issues with display rotation
# and touchscreen calibration on Pi 4 Bookworm.
exec xinit /home/pi/kiln_controller/xinitrc.sh -- :0 vt1 -nolisten tcp
KIOSK
chmod +x /home/pi/kiln_controller/start_kiosk.sh
chown pi:pi /home/pi/kiln_controller/start_kiosk.sh

# TTY1 auto-login for pi
mkdir -p /etc/systemd/system/getty@tty1.service.d
cat > /etc/systemd/system/getty@tty1.service.d/autologin.conf << 'AUTOLOGIN'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin pi --noclear %I $TERM
AUTOLOGIN

# Disable all display managers and boot to text target
systemctl disable lightdm 2>/dev/null || true
systemctl disable gdm     2>/dev/null || true
systemctl disable sddm    2>/dev/null || true
systemctl set-default multi-user.target

# Remove old kiln-kiosk systemd service if present from previous install
systemctl disable kiln-kiosk.service 2>/dev/null || true
rm -f /etc/systemd/system/kiln-kiosk.service
systemctl daemon-reload

# Add kiosk launcher to .bash_profile — only fires on TTY1, not SSH
PROFILE=/home/pi/.bash_profile
if [ -f "$PROFILE" ]; then
  sed -i '/# STEELHAUS kiosk/,/# end STEELHAUS kiosk/d' "$PROFILE"
fi
cat >> "$PROFILE" << 'PROFILE_BLOCK'
# STEELHAUS kiosk — launch when logging into TTY1
if [ "$(tty)" = "/dev/tty1" ]; then
  exec /home/pi/kiln_controller/start_kiosk.sh
fi
# end STEELHAUS kiosk
PROFILE_BLOCK
chown pi:pi "$PROFILE"

# Verify kiosk scripts exist and are executable
KIOSK_OK=true
for f in /home/pi/kiln_controller/start_kiosk.sh \
          /home/pi/kiln_controller/xinitrc.sh; do
  if [ ! -x "$f" ]; then
    echo "      WARNING: $f missing or not executable"
    ERRORS="$ERRORS\n  - Kiosk script missing: $f"
    KIOSK_OK=false
  fi
done
if [ "$KIOSK_OK" = true ]; then
  echo "      Kiosk scripts verified."
fi

echo "      TTY1 auto-login + .bash_profile kiosk launch configured."
echo "      Boot: kernel -> TTY1 auto-login -> xinit -> Chromium."
echo "      SSH access unaffected."

# Display rotation is handled via xrandr --rotate left inside xinitrc.sh.
# display_rotate= is ignored by the vc4-kms-v3d KMS driver on Bookworm
# and must NOT be set — remove it if present from a previous install.
CONFIG=/boot/firmware/config.txt
if [ -f "$CONFIG" ]; then
  sed -i '/^display_rotate=/d' "$CONFIG"
  echo "      Removed display_rotate from config.txt (not used with KMS driver)."
fi

# ── 5a. Create blank cursor theme ────────────────────────────────────────────
echo "[5a] Creating blank cursor theme..."
apt-get install -y python3 2>/dev/null
mkdir -p /usr/share/icons/blank/cursors
cat > /usr/share/icons/blank/index.theme << 'THEME'
[Icon Theme]
Name=blank
Comment=Invisible cursor
THEME

# Write a valid minimal Xcursor file with a 1x1 fully transparent image
python3 << 'PYEOF'
import struct

def pack_xcursor():
    # TOC entry points to offset 20 (right after 20-byte file header)
    toc_offset = 20
    # Image chunk: 36 bytes header + 4 bytes pixel data = 40 bytes
    img_header_size = 36
    width = height = 1
    xhot = yhot = 0
    delay = 50  # ms

    toc = struct.pack('<III', 0xfffd0002, 1, toc_offset)  # type, subtype(size), offset

    img = struct.pack('<IIIIIIIII',
        img_header_size,  # chunk header size
        0xfffd0002,       # type: image
        1,                # subtype: nominal size
        1,                # version
        width, height,
        xhot, yhot,
        delay,
    )
    img += struct.pack('<I', 0x00000000)  # 1 pixel, fully transparent ARGB

    # File header: magic(4) + header_size(4) + version(4) + ntoc(4) = 16 bytes
    file_header = b'Xcur' + struct.pack('<III', 16, 1, 1)

    data = file_header + toc + img
    for name in ['default', 'left_ptr', 'pointer', 'crosshair', 'text', 'wait',
                 'move', 'n-resize', 's-resize', 'e-resize', 'w-resize',
                 'ne-resize', 'nw-resize', 'se-resize', 'sw-resize']:
        open(f'/usr/share/icons/blank/cursors/{name}', 'wb').write(data)

pack_xcursor()
print("Blank cursor files written.")
PYEOF
echo "      Blank cursor theme created."

# Set cursor theme via GTK settings (X11)
mkdir -p /home/pi/.config/gtk-3.0
cat > /home/pi/.config/gtk-3.0/settings.ini << 'GTK'
[Settings]
gtk-cursor-theme-name=blank
gtk-cursor-theme-size=24
GTK
chown -R pi:pi /home/pi/.config/gtk-3.0

# Also set via X cursor config as fallback
mkdir -p /home/pi/.icons/default
cat > /home/pi/.icons/default/index.theme << 'XTHEME'
[Icon Theme]
Name=Default
Comment=Default cursor
Inherits=blank
XTHEME
chown -R pi:pi /home/pi/.icons
echo "      GTK cursor theme set to blank."

# ── 5b. Touch input calibration udev rule ────────────────────────────────────
# Applies a 90° CCW libinput calibration matrix to any USB touchscreen.
# Matches on USB vendor ID and common name patterns rather than a specific
# device name, so it works with yldzkj, Waveshare, Elecrow, and other
# common 7" HDMI touchscreens. The matrix "0 -1 1 1 0 0" rotates raw touch
# coordinates 90° CCW to match the xrandr --rotate left display orientation.
cat > /etc/udev/rules.d/99-steelhaus-touch.rules << 'UDEV'
# Rotate touch input 90° CCW for any USB touchscreen (portrait display)

# Match by USB vendor ID 222a (covers yldzkj and many compatible chips)
ACTION=="add|change", KERNEL=="event*", \
  ATTRS{idVendor}=="222a", \
  ENV{LIBINPUT_CALIBRATION_MATRIX}="0 -1 1 1 0 0", \
  ENV{ID_INPUT_MOUSE}="", \
  ENV{ID_INPUT_MICE}=""

# Match by CTP in device name (common across many touchscreen brands)
ACTION=="add|change", KERNEL=="event*", \
  ATTRS{name}=="*CTP*", \
  ENV{LIBINPUT_CALIBRATION_MATRIX}="0 -1 1 1 0 0", \
  ENV{ID_INPUT_MOUSE}="", \
  ENV{ID_INPUT_MICE}=""

# Match by touchscreen in device name (Waveshare, Elecrow, etc.)
ACTION=="add|change", KERNEL=="event*", \
  ATTRS{name}=="*[Tt]ouchscreen*", \
  ENV{LIBINPUT_CALIBRATION_MATRIX}="0 -1 1 1 0 0", \
  ENV{ID_INPUT_MOUSE}="", \
  ENV{ID_INPUT_MICE}=""

# Suppress mouse node for CTP devices
ACTION=="add|change", KERNEL=="mouse*", \
  ATTRS{name}=="*CTP*", \
  ENV{ID_INPUT_MOUSE}="", \
  ENV{ID_INPUT_MICE}=""
UDEV
udevadm control --reload-rules
udevadm trigger
echo "      Touch calibration udev rule installed (matches any USB touchscreen)."

# ── 6. Configure firewall ─────────────────────────────────────────────────────
echo "[6/8] Configuring firewall..."
if command -v ufw >/dev/null 2>&1; then
  ufw allow 5000/tcp comment "STEELHAUS UI" 2>/dev/null || true
  echo "      Opened port 5000."
else
  echo "      UFW not present — skipping."
fi

# ── 7. Install Tailscale ──────────────────────────────────────────────────────
echo "[7/8] Installing Tailscale..."
if command -v tailscale >/dev/null 2>&1; then
  echo "      Already installed."
else
  curl -fsSL https://tailscale.com/install.sh | sh 2>/dev/null
  echo "      Installed."
fi
systemctl enable tailscaled 2>/dev/null || true
systemctl start  tailscaled 2>/dev/null || true
echo "      Tailscale service enabled (always-on)."

# ── 8. Final permissions ──────────────────────────────────────────────────────
echo "[8/8] Setting permissions..."
chown -R pi:pi /home/pi/kiln_controller
chmod +x /home/pi/kiln_controller/start_kiosk.sh
echo "      Done."

# ── Summary ───────────────────────────────────────────────────────────────────
PI_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "unknown")

echo ""
echo "=============================================="
echo "  Setup Complete!"
echo "=============================================="
if [ -n "$ERRORS" ]; then
  echo ""
  echo "  WARNINGS (non-fatal):"
  echo -e "$ERRORS"
fi
echo ""
echo "  Web UI: http://${PI_IP}:5000"
echo "  Also:   http://kiln.local:5000"
echo ""
echo "  NEXT STEPS:"
echo "  1. Plug in the ESP32"
echo "  2. sudo systemctl start kiln-controller"
echo "  3. sudo systemctl status kiln-controller"
echo "  4. sudo reboot"
echo ""
echo "  After reboot: Chromium opens automatically on the display."
echo "  SSH access still works normally."
echo ""
echo "  NOTE: Display is configured to rotate 90 degrees CCW (portrait,"
echo "  connector at bottom) via xrandr inside xinitrc.sh."
echo "  To change orientation, edit /home/pi/kiln_controller/xinitrc.sh"
echo "  and change '--rotate left' to: normal, right, inverted, or left."
echo "=============================================="
echo ""
