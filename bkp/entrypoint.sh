#!/bin/bash

# VNC Configuration
DISPLAY_NUM="${DISPLAY_NUM:-1}"
export DISPLAY=:$DISPLAY_NUM
VNC_RESOLUTION="${VNC_RESOLUTION:-1920x1080}"
VNC_PASSWORD="${VNC_PASSWORD:-vncpassword}"
NOVNC_PORT="${NOVNC_PORT:-6080}"

echo "Starting noVNC setup..."

# Setup VNC password
mkdir -p ~/.vnc
echo "$VNC_PASSWORD" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Kill ALL existing VNC/X servers to ensure clean start
pkill -9 Xtigervnc 2>/dev/null || true
pkill -9 fluxbox 2>/dev/null || true
rm -rf /tmp/.X11-unix/X${DISPLAY_NUM} 2>/dev/null || true
rm -rf /tmp/.X${DISPLAY_NUM}-lock 2>/dev/null || true
vncserver -kill :${DISPLAY_NUM} 2>/dev/null || true
sleep 2

# Create minimal xstartup
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
/usr/bin/fluxbox
EOF
chmod +x ~/.vnc/xstartup

echo "Starting VNC server on display :${DISPLAY_NUM}..."

# Start VNC server
vncserver :$DISPLAY_NUM \
    -geometry $VNC_RESOLUTION \
    -depth 24 \
    -localhost no \
    -SecurityTypes VncAuth \
    -rfbauth ~/.vnc/passwd \
    -AlwaysShared

if [ $? -ne 0 ]; then
    echo "ERROR: VNC server failed to start!"
    echo "Log contents:"
    cat ~/.vnc/*.log
    exit 1
fi

# Wait for VNC to be ready
sleep 5

# Verify VNC is running
if ! pgrep -f "Xtigervnc.*:${DISPLAY_NUM}" > /dev/null; then
    echo "ERROR: VNC server not running after start!"
    exit 1
fi

echo "VNC server started successfully"

# Start noVNC
echo "Starting noVNC on port ${NOVNC_PORT}..."
nohup /opt/noVNC/utils/novnc_proxy \
    --vnc localhost:$((5900 + DISPLAY_NUM)) \
    --listen $NOVNC_PORT \
    > /tmp/novnc.log 2>&1 &

sleep 2

echo "================================================"
echo "✓ noVNC started: http://localhost:${NOVNC_PORT}"
echo "✓ VNC password: ${VNC_PASSWORD}"
echo "✓ Display: ${DISPLAY}"
echo "================================================"

exit 0