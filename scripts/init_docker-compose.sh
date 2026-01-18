#!/bin/bash
## Generate docker-compose.yml for multi-robot ROS2 setup

set -e

OUTPUT_FILE="tmp/docker-compose.yml"
CONFIG_FILE="${1:-../setup.ini}"

declare -A CONFIG
declare -a ROBOTS
declare -A NETWORKS
declare -A VOLUMES

# Enhanced INI parser - FIXED to handle files without trailing newline
parse_ini() {
    local section=""
    local line_num=0

    # Read line by line, handling files without trailing newlines
    while IFS= read -r raw_line || [ -n "$raw_line" ]; do
        line_num=$((line_num + 1))

        # Show raw line for debugging
        echo "Line $line_num (raw): >>>$raw_line<<<"

        # Skip comments
        if [[ "$raw_line" =~ ^[[:space:]]*# ]]; then
            echo "  -> Skipped (comment)"
            continue
        fi

        # Skip empty lines
        if [[ "$raw_line" =~ ^[[:space:]]*$ ]]; then
            echo "  -> Skipped (empty)"
            continue
        fi

        # Remove leading/trailing whitespace
        local line="${raw_line#"${raw_line%%[![:space:]]*}"}"
        line="${line%"${line##*[![:space:]]}"}"

        echo "  -> Trimmed: >>>$line<<<"

        # Check for section header [name]
        if [[ "$line" =~ ^\[([^]]+)\]$ ]]; then
            section="${BASH_REMATCH[1]}"
            ROBOTS+=("$section")
            echo "  -> Found section: [$section]"
            continue
        fi

        # Check for key=value
        if [[ "$line" =~ ^([^=]+)=(.*)$ ]]; then
            local key="${BASH_REMATCH[1]}"
            local value="${BASH_REMATCH[2]}"

            # Trim key and value
            key="${key#"${key%%[![:space:]]*}"}"
            key="${key%"${key##*[![:space:]]}"}"
            value="${value#"${value%%[![:space:]]*}"}"
            value="${value%"${value##*[![:space:]]}"}"

            # Store
            CONFIG["${section}_${key}"]="$value"
            echo "  -> Stored: ${section}_${key} = >>>$value<<<"
        else
            echo "  -> WARNING: Line doesn't match pattern"
        fi
    done < "$CONFIG_FILE"
}

echo "========================================"
echo "Parsing $CONFIG_FILE"
echo "========================================"
parse_ini
echo ""
echo "Found ${#ROBOTS[@]} robot(s): ${ROBOTS[*]}"
echo ""

echo "--- Full Configuration ---"
for key in "${!CONFIG[@]}"; do
    echo "$key = ${CONFIG[$key]}"
done
echo "--------------------------"
echo ""

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
    PROFILE=${CONFIG[${ROBOT_NAME}_install_profile]:-custom}
    ROS_DISTRO=${CONFIG[${ROBOT_NAME}_ros_distro]:-jazzy}
    ROS_INSTALL=${CONFIG[${ROBOT_NAME}_ros_install]:-osrf/ros:${ROS_DISTRO}-desktop-full}
    DOMAIN=${CONFIG[${ROBOT_NAME}_ros_domain]:-0}
    VOLS=${CONFIG[${ROBOT_NAME}_volumes]}
    ENDFUNCTION=${CONFIG[${ROBOT_NAME}_endfunction]}
    ENV_VARS=${CONFIG[${ROBOT_NAME}_env]}

    echo "==================================="
    echo "Processing: $ROBOT_NAME"
    echo "==================================="
    echo "DISPLAY: $DISPLAY"
    echo "NETWORK: $NETWORK"
    echo "PROFILE: $PROFILE"
    echo "DOMAIN: $DOMAIN"
    echo "VOLS: $VOLS"
    echo "ENDFUNCTION: $ENDFUNCTION"
    echo "ENV_VARS: >>>$ENV_VARS<<<"
    echo ""

    # Calculate ports
    DISPLAY_NUM=${DISPLAY}
    VNC_PORT=$((5900 + DISPLAY_NUM))
    NOVNC_PORT=$((6080 + DISPLAY_NUM - 1))

    # Track resources
    [ -n "$NETWORK" ] && NETWORKS[$NETWORK]=1
    if [ -n "$VOLS" ]; then
        IFS=',' read -ra VOL_ARRAY <<< "$VOLS"
        for vol in "${VOL_ARRAY[@]}"; do
            vol="${vol#"${vol%%[![:space:]]*}"}"
            vol="${vol%"${vol##*[![:space:]]}"}"
            [ -n "$vol" ] && VOLUMES["${ROBOT_NAME}_${vol}"]="$vol"
        done
    fi

    # Copy endfunction per robot
    TARGET_DIR="./tmp/${ROBOT_NAME}"
    mkdir -p "$TARGET_DIR"

    if [ -n "$ENDFUNCTION" ]; then
        SRC="../$ENDFUNCTION"
        if [ -f "$SRC" ]; then
            cp "$SRC" "$TARGET_DIR/endfunction.sh"
            echo "✓ Copied endfunction: $SRC -> $TARGET_DIR/endfunction.sh"
        else
            echo "⚠ Endfunction not found: $SRC (using template)"
            cp template/endfunction.sh "$TARGET_DIR/endfunction.sh"
        fi
    else
        echo "ℹ No endfunction specified (using template)"
        cp template/endfunction.sh "$TARGET_DIR/endfunction.sh"
    fi

    echo "Generating docker-compose entry for $ROBOT_NAME..."

    cat >> "$OUTPUT_FILE" <<EOF

  $ROBOT_NAME:
    container_name: $ROBOT_NAME
    build:
      context: .
      dockerfile: ../Dockerfile
      args:
        ROS_DISTRO: $ROS_DISTRO
        ROS_INSTALL: $ROS_INSTALL
        USERNAME: ubuntu
        USER_UID: 1000
        USER_GID: 1000
        INSTALL_PROFILE: $PROFILE
        ROBOT_NAME: $ROBOT_NAME

    environment:
      DISPLAY: ":$DISPLAY"
      ROS_DISTRO: "$ROS_DISTRO"
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

    # Process custom environment variables
    if [ -n "$ENV_VARS" ]; then
        echo "Processing custom environment variables..."
        echo "ENV_VARS content: >>>$ENV_VARS<<<"

        IFS=';' read -ra ENV_ARRAY <<< "$ENV_VARS"
        echo "Split into ${#ENV_ARRAY[@]} parts"

        for env_kv in "${ENV_ARRAY[@]}"; do
            # Trim whitespace
            env_kv="${env_kv#"${env_kv%%[![:space:]]*}"}"
            env_kv="${env_kv%"${env_kv##*[![:space:]]}"}"

            echo "  Processing: >>>$env_kv<<<"

            # Split on first colon
            if [[ "$env_kv" =~ ^([^:]+):(.*)$ ]]; then
                KEY="${BASH_REMATCH[1]}"
                VALUE="${BASH_REMATCH[2]}"

                # Trim key and value
                KEY="${KEY#"${KEY%%[![:space:]]*}"}"
                KEY="${KEY%"${KEY##*[![:space:]]}"}"
                VALUE="${VALUE#"${VALUE%%[![:space:]]*}"}"
                VALUE="${VALUE%"${VALUE##*[![:space:]]}"}"

                if [ -n "$KEY" ]; then
                    echo "      $KEY: \"$VALUE\"" >> "$OUTPUT_FILE"
                    echo "    ✓ Added: $KEY=$VALUE"
                fi
            else
                echo "    ✗ Invalid format (expected KEY:VALUE)"
            fi
        done
    else
        echo "No custom environment variables"
    fi
    echo ""

    # Add networks
    if [ -n "$NETWORK" ]; then
        cat >> "$OUTPUT_FILE" <<EOF

    networks:
      - $NETWORK
EOF
    fi

    # Add volumes
    cat >> "$OUTPUT_FILE" <<EOF

    volumes:
      - ../../../src:/home/ubuntu/ros2_ws/src
      - ../../../launch:/home/ubuntu/ros2_ws/launch
EOF

    if [ -n "$VOLS" ]; then
        IFS=',' read -ra VOL_ARRAY <<< "$VOLS"
        for vol in "${VOL_ARRAY[@]}"; do
            vol="${vol#"${vol%%[![:space:]]*}"}"
            vol="${vol%"${vol##*[![:space:]]}"}"
            if [ -n "$vol" ]; then
                echo "      - ${ROBOT_NAME}_${vol}:/home/ubuntu/ros2_ws/${vol}" >> "$OUTPUT_FILE"
            fi
        done
    fi

    # Add ports
    cat >> "$OUTPUT_FILE" <<EOF

    ports:
      - "$NOVNC_PORT:$NOVNC_PORT"
      - "$VNC_PORT:$VNC_PORT"
EOF

done

# Add volumes section
if [ ${#VOLUMES[@]} -gt 0 ]; then
    echo "" >> "$OUTPUT_FILE"
    echo "volumes:" >> "$OUTPUT_FILE"
    for vol in "${!VOLUMES[@]}"; do
        echo "  $vol:" >> "$OUTPUT_FILE"
    done
fi

# Add networks section
if [ ${#NETWORKS[@]} -gt 0 ]; then
    echo "" >> "$OUTPUT_FILE"
    echo "networks:" >> "$OUTPUT_FILE"
    for net in "${!NETWORKS[@]}"; do
        echo "  $net:" >> "$OUTPUT_FILE"
        echo "    driver: bridge" >> "$OUTPUT_FILE"
    done
fi

echo ""
echo "========================================="
echo "✓ Generated $OUTPUT_FILE"
echo "========================================="
echo ""
echo "Verify environment variables:"
echo "  grep -A 25 'environment:' $OUTPUT_FILE | grep -A 15 robot1"