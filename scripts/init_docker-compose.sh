#!/bin/bash
## Generate docker-compose.yml for multi-robot ROS2 setup

set -e


OUTPUT_FILE="tmp/docker-compose.yml"
CONFIG_FILE="${1:-../setup.ini}"

declare -A CONFIG
declare -a ROBOTS
declare -A NETWORKS
declare -A VOLUMES

# Simple INI parser
parse_ini() {
    local section=""
    while IFS='=' read -r key value; do
        # Skip comments and empty lines
        [[ "$key" =~ ^[[:space:]]*# ]] && continue
        [[ -z "$key" ]] && continue

        # Trim whitespace
        key=$(echo "$key" | xargs)
        value=$(echo "$value" | xargs)

        # Check for section header [name]
        if [[ "$key" =~ ^\[(.+)\]$ ]]; then
            section="${BASH_REMATCH[1]}"
            ROBOTS+=("$section")
        else
            # Store as section_key=value
            CONFIG["${section}_${key}"]="$value"
        fi
    done < "$CONFIG_FILE"
}

echo "Parsing $CONFIG_FILE"
parse_ini
echo "Found ${#ROBOTS[@]} robot(s): ${ROBOTS[*]}"

echo "--- Full Configuration ---"
for key in "${!CONFIG[@]}"; do
    echo "Key: $key | Value: ${CONFIG[$key]}"
done
echo "--------------------------"

display_counter=0
# Start output

cat > "$OUTPUT_FILE" <<'EOF'
# Auto-generated docker-compose.yml
# Generated from setup.ini

services:
EOF

# Generate each robot
for ROBOT_NAME in "${ROBOTS[@]}"; do
    display_counter=$((display_counter+1))
    DISPLAY=${display_counter}
    NETWORK=${CONFIG[${ROBOT_NAME}_network]}
    PROFILE=${CONFIG[${ROBOT_NAME}_profile]:-custom}
    DOMAIN=${CONFIG[${ROBOT_NAME}_domain]:-0}
    VOLS=${CONFIG[${ROBOT_NAME}_volumes]}
    ENDFUNCTION=${CONFIG[${ROBOT_NAME}_endfunction]}

    echo $DISPLAY $NETWORK $PROFILE $DOMAIN $VOLS

    # Calculate ports
    DISPLAY_NUM=${DISPLAY}
    VNC_PORT=$((5900 + DISPLAY_NUM))
    NOVNC_PORT=$((6080 + DISPLAY_NUM - 1))

    echo $DISPLAY_NUM $VNC_PORT $NOVNC_PORT

    # Track resources
    [ -n "$NETWORK" ] && NETWORKS[$NETWORK]=1
    if [ -n "$VOLS" ]; then
        IFS=',' read -ra VOL_ARRAY <<< "$VOLS"
        for vol in "${VOL_ARRAY[@]}"; do
            vol=$(echo "$vol" | xargs)
            [ -n "$vol" ] && VOLUMES[$vol]=1
        done
    fi

    # Copy endfunction per robot
    TARGET_DIR="./tmp/${ROBOT_NAME}"
    mkdir -p "$TARGET_DIR"

    if [ -n "$ENDFUNCTION" ]; then
        SRC="../$ENDFUNCTION"

        if [ -f "$SRC" ]; then
            cp "$SRC" "$TARGET_DIR/endfunction.sh"
            echo "✓ Copied endfunction for $ROBOT_NAME -> $TARGET_DIR/endfunction.sh"
        else
            echo "⚠ Endfunction not found: $SRC"
            echo "  Using template instead."
            cp template/endfunction.sh "$TARGET_DIR/endfunction.sh"
        fi
    else
        echo "ℹ No endfunction specified for $ROBOT_NAME, using template."
        cp template/endfunction.sh "$TARGET_DIR/endfunction.sh"
    fi

    echo "  Generating: $ROBOT_NAME"

    cat >> "$OUTPUT_FILE" <<EOF

  $ROBOT_NAME:
    container_name: $ROBOT_NAME
    build:
      dockerfile: ../Dockerfile
      args:
        ROS_DISTRO: jazzy
        USERNAME: ubuntu
        USER_UID: 1000
        USER_GID: 1000
        INSTALL_PROFILE: $PROFILE
        ROBOT_NAME: $ROBOT_NAME

    environment:
      DISPLAY: ":$DISPLAY"
      ROS_DISTRO: "jazzy"
      ROS_DOMAIN_ID: $DOMAIN
      PYTHONUNBUFFERED: "1"
      LIBGL_ALWAYS_SOFTWARE: "1"
      MESA_GL_VERSION_OVERRIDE: "3.3"
      QT_X11_NO_MITSHM: "1"
      QT_QPA_PLATFORM: "xcb"
      GZ_SIM_RENDER_ENGINE: "ogre"
      GZ_PARTITION: ros2_gz_partition
      ROBOT_NAME: $ROBOT_NAME
EOF

    if [ -n "$NETWORK" ]; then
        cat >> "$OUTPUT_FILE" <<EOF

    networks:
      - $NETWORK
EOF
    fi

    cat >> "$OUTPUT_FILE" <<EOF

    volumes:
      - ../../src:/home/ubuntu/ros2_ws/src
      - ../../launch:/home/ubuntu/ros2_ws/launch
EOF

    if [ -n "$VOLS" ]; then
        IFS=',' read -ra VOL_ARRAY <<< "$VOLS"
        for vol in "${VOL_ARRAY[@]}"; do
            vol=$(echo "$vol" | xargs)
            [ -n "$vol" ] && echo "      - ${vol}:/home/ubuntu/ros2_ws/${vol}" >> "$OUTPUT_FILE"
        done
    fi

    cat >> "$OUTPUT_FILE" <<EOF

    ports:
      - "$NOVNC_PORT:$NOVNC_PORT"
      - "$VNC_PORT:$VNC_PORT"

    command: /tmp/endfunction.sh
EOF

done

# Add volumes
if [ ${#VOLUMES[@]} -gt 0 ]; then
    echo "" >> "$OUTPUT_FILE"
    echo "volumes:" >> "$OUTPUT_FILE"
    for vol in "${!VOLUMES[@]}"; do
        echo "  $vol:" >> "$OUTPUT_FILE"
    done
fi

# Add networks
if [ ${#NETWORKS[@]} -gt 0 ]; then
    echo "" >> "$OUTPUT_FILE"
    echo "networks:" >> "$OUTPUT_FILE"
    for net in "${!NETWORKS[@]}"; do
        echo "  $net:" >> "$OUTPUT_FILE"
        echo "    driver: bridge" >> "$OUTPUT_FILE"
    done
fi

echo ""
echo "✓ Generated $OUTPUT_FILE"