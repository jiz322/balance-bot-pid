echo "[3/5] Creating udev rules for USB serial ports..."

UDEV_FILE="/etc/udev/rules.d/99-balancer-usb.rules"
cat > "$UDEV_FILE" << 'EOF'
# Balancer Robot USB devices – grant access to 'dialout' group
# CAN2USB adapters (edit VID/PID to match your hardware)
# CH340
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="can2usb"
# CP2102
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="can2usb"
# FTDI
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="can2usb"

# Generic: all USB-serial ports accessible without sudo
SUBSYSTEM=="tty", ATTRS{idVendor}=="*", KERNEL=="ttyUSB*", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="*", KERNEL=="ttyACM*", MODE="0666"

# Lower FTDI latency timer automatically
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
EOF

udevadm control --reload-rules
udevadm trigger
echo "  → Created $UDEV_FILE"

# ── 4. Add current user to dialout group ─────────────────────
echo ""
echo "[4/5] Adding user to dialout group..."
REAL_USER="${SUDO_USER:-$USER}"
usermod -aG dialout "$REAL_USER"
echo "  → Added '$REAL_USER' to dialout group (re-login to take effect)"

# ── 5. Optional: RT scheduling capability ────────────────────
echo ""
echo "[5/5] Enabling real-time scheduling for user..."

RT_CONF="/etc/security/limits.d/99-balancer-rt.conf"
cat > "$RT_CONF" << EOF
# Allow balancer user to use SCHED_FIFO
$REAL_USER  -  rtprio  50
$REAL_USER  -  nice    -20
$REAL_USER  -  memlock unlimited
EOF
echo "  → Created $RT_CONF"

echo ""
echo "======================================"
echo "  Setup complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo "  1. Re-login (or reboot) for group/RT changes to apply"
echo "  2. Plug in both USB devices"
echo "  3. Run:  python3 tools/pi5_usb_helper.py       to check ports"
echo "  4. Run:  python3 drivers/imu_driver_6_400hz.py  to test IMU"
echo "  5. Run:  python3 drivers/robstride_driver.py    to test motors"
echo "  6. Run:  python3 control/classical_mit_400hz.py"
echo ""
echo "If ports are swapped, either:"
echo "  - Use --motor-port /dev/ttyUSBx --imu-port /dev/ttyUSBy"
echo "  - Edit VID/PID in tools/pi5_usb_helper.py for auto-detect"
echo ""

