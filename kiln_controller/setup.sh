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
# Install cage 0.1.4 specifically — 0.2.0 from the RPi repo broke XCURSOR_THEME support
# and the cursor cannot be hidden on that version.
apt-get install -y \
  python3 python3-pip \
  sqlite3 \
  curl \
  chromium-browser \
  2>/dev/null
apt-get install -y --allow-downgrades cage=0.1.4-4 2>/dev/null || apt-get install -y cage 2>/dev/null
# Pin cage to prevent auto-upgrade back to 0.2.0
echo "cage hold" | dpkg --set-selections
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

# ── 5. Configure direct-boot kiosk (cage + Chromium, no desktop) ─────────────
echo "[5/8] Configuring direct-boot kiosk (cage — no desktop)..."
#
# Boot flow:
#   kernel -> getty auto-login as pi on TTY1 -> .bash_profile runs start_kiosk.sh
#   -> cage launches Chromium as the only app, full screen.
#
# Launching cage from .bash_profile is the most reliable method — all seat,
# DRM, and Wayland environment variables are set correctly by the login process.
# SSH sessions are NOT affected (tty check guards it).

# Create kiosk launch script
# Install wlr-randr for Wayland display rotation
apt-get install -y wlr-randr 2>/dev/null || true

# Create a wrapper script that cage will run — it rotates the display first,
# then launches Chromium. Both run inside the same cage Wayland session.
cat > /home/pi/kiln_controller/cage_wrapper.sh << 'WRAPPER'
#!/bin/bash
# Rotate display 270 degrees, then launch Chromium
# Display rotation is handled at the DRM/kernel level via config.txt (display_rotate=1)
# wlr-randr not used — cage 0.1.4 does not support wlr-output-management protocol.

# Apply touch input calibration matrix to match 90-degree display rotation.
# Device: yldzkj USB2IIC_CTP_CONTROL (/dev/input/event4)
# Matrix for 90-degree clockwise rotation: 0 -1 1 1 0 0 0 0 1
TOUCH_DEV="yldzkj USB2IIC_CTP_CONTROL"
if command -v xinput >/dev/null 2>&1; then
  xinput set-prop "$TOUCH_DEV" "libinput Calibration Matrix" 0 -1 1 1 0 0 0 0 1 2>/dev/null || true
fi
# Wayland/libinput environment variable approach (works without xinput)
export LIBINPUT_CALIBRATION_MATRIX="0 -1 1 1 0 0"

exec chromium-browser \
  --hide-scrollbars \
  --kiosk \
  --noerrdialogs \
  --disable-infobars \
  --no-first-run \
  --disable-restore-session-state \
  --disable-session-crashed-bubble \
  --disable-translate \
  --disable-features=TranslateUI \
  --enable-features=UseOzonePlatform \
  --ozone-platform=wayland \
  --check-for-update-interval=31536000 \
  http://localhost:5000
WRAPPER
chmod +x /home/pi/kiln_controller/cage_wrapper.sh
chown pi:pi /home/pi/kiln_controller/cage_wrapper.sh

cat > /home/pi/kiln_controller/start_kiosk.sh << 'KIOSK'
#!/bin/bash
# STEELHAUS Kiosk — X11 + xrandr rotation + Chromium

# Wait for Flask to be ready (up to 60s)
for i in $(seq 1 30); do
  if curl -s http://localhost:5000 > /dev/null 2>&1; then
    break
  fi
  sleep 2
done

# Start X server on VT1, rotate display 90 degrees clockwise
xinit /home/pi/kiln_controller/xinitrc.sh -- :0 vt1 -nolisten tcp &
KIOSK

chmod +x /home/pi/kiln_controller/start_kiosk.sh
chown pi:pi /home/pi/kiln_controller/start_kiosk.sh

# Create xinitrc — X session script that rotates display and launches Chromium
cat > /home/pi/kiln_controller/xinitrc.sh << 'XINIT'
#!/bin/bash
# Rotate display 90 degrees clockwise via xrandr (right = 90 CW)
xrandr --output HDMI-1 --rotate right 2>/dev/null ||   xrandr --output HDMI-A-1 --rotate right 2>/dev/null || true

# Hide cursor after 0.1s idle
unclutter -idle 0.1 -root &

# Matchbox window manager — no title bars, fullscreen support
matchbox-window-manager -use_titlebar no &

# Launch Chromium in kiosk mode (X11 — no Wayland flags)
exec chromium-browser   --kiosk   --noerrdialogs   --disable-infobars   --no-first-run   --disable-restore-session-state   --disable-session-crashed-bubble   --disable-translate   --disable-features=TranslateUI   --hide-scrollbars   --check-for-update-interval=31536000   http://localhost:5000
XINIT
chmod +x /home/pi/kiln_controller/xinitrc.sh
chown pi:pi /home/pi/kiln_controller/xinitrc.sh

# TTY1 auto-login for pi
mkdir -p /etc/systemd/system/getty@tty1.service.d
cat > /etc/systemd/system/getty@tty1.service.d/autologin.conf << 'AUTOLOGIN'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin pi --noclear %I $TERM
AUTOLOGIN

# Disable display manager
systemctl disable lightdm 2>/dev/null || true
systemctl disable gdm     2>/dev/null || true
systemctl set-default multi-user.target

# Remove old kiln-kiosk systemd service if present from previous install
systemctl disable kiln-kiosk.service 2>/dev/null || true
rm -f /etc/systemd/system/kiln-kiosk.service
systemctl daemon-reload

# Add cage launcher to pi's .bash_profile — only fires on TTY1, not SSH
PROFILE=/home/pi/.bash_profile
# Remove any previous kiosk block cleanly
if [ -f "$PROFILE" ]; then
  sed -i '/# STEELHAUS kiosk/,/# end STEELHAUS kiosk/d' "$PROFILE"
fi
# Append kiosk block
cat >> "$PROFILE" << 'PROFILE_BLOCK'
# STEELHAUS kiosk — launch cage+Chromium when logging into TTY1
if [ "$(tty)" = "/dev/tty1" ]; then
  exec /home/pi/kiln_controller/start_kiosk.sh
fi
# end STEELHAUS kiosk
PROFILE_BLOCK
chown pi:pi "$PROFILE"

echo "      TTY1 auto-login + .bash_profile kiosk launch configured."

# Rotate display 90 degrees at the DRM level (works with any Wayland compositor)
CONFIG=/boot/firmware/config.txt
if [ -f "$CONFIG" ]; then
  # Remove any existing display_rotate line and add the correct one
  sed -i '/^display_rotate=/d' "$CONFIG"
  echo "display_rotate=1" >> "$CONFIG"
  echo "      Display rotation set to 90 degrees in config.txt."
else
  echo "      WARNING: /boot/firmware/config.txt not found — set display_rotate=1 manually."
fi
echo "      Boot: kernel -> TTY1 auto-login -> cage -> Chromium."
echo "      SSH access unaffected."

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

# cage 0.2.0 reads cursor theme from GTK settings, not XCURSOR_THEME env var
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
# This applies the 90-degree calibration matrix at the kernel/udev level,
# before any compositor sees the device — most reliable method.
cat > /etc/udev/rules.d/99-steelhaus-touch.rules << 'UDEV'
ACTION=="add|change", KERNEL=="event*", \
  ATTRS{name}=="yldzkj USB2IIC_CTP_CONTROL", \
  ENV{LIBINPUT_CALIBRATION_MATRIX}="0 -1 1 1 0 0", \
  ENV{ID_INPUT_MOUSE}="", \
  ENV{ID_INPUT_MICE}=""

# Also suppress the mice/mouse0 node for this device
ACTION=="add|change", KERNEL=="mouse*", \
  ATTRS{name}=="yldzkj USB2IIC_CTP_CONTROL", \
  ENV{ID_INPUT_MOUSE}="", \
  ENV{ID_INPUT_MICE}=""
UDEV
udevadm control --reload-rules
udevadm trigger
echo "      Touch calibration udev rule installed."

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
echo "  After reboot: Chromium opens in ~5-8 seconds."
echo "  No desktop environment ever appears."
echo "  SSH access still works normally."
echo "=============================================="
echo ""
