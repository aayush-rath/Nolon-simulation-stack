#!/bin/bash

# Setup script for Nolon Bot Docker environment
# This script prepares the Docker environment for GUI applications and sets up X11 forwarding

set -e

echo "Setting up Nolon Bot Docker environment..."

# Create necessary directories
mkdir -p maps logs

# Set up X11 forwarding for GUI applications
echo "Setting up X11 forwarding..."

# Allow docker to access X11
xhost +local:docker

# Create X11 authentication file
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    if [ -n "$xauth_list" ]; then
        echo "$xauth_list" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Set proper permissions for directories
echo "Setting up directory permissions..."
sudo chown -R $USER:$USER maps logs
chmod 755 maps logs

# Create .env file if it doesn't exist
if [ ! -f .env ]; then
    echo "Creating .env file..."
    cat << EOF > .env
USER_ID=$(id -u)
GROUP_ID=$(id -g)
ROS_DOMAIN_ID=0
DISPLAY=$DISPLAY
EOF
    echo "Created .env file with user configuration"
fi

echo "Docker environment setup complete!"
echo ""
echo "Usage examples:"
echo "  Basic simulation:     docker compose up"
echo "  With navigation:      docker compose --profile navigation up"
echo "  With mapping:         docker compose --profile mapping up"
echo "  With manipulation:    docker compose --profile manipulation up"
echo "  With visualization:   docker compose --profile visualization up"
echo "  Development mode:     docker compose --profile development up nolon-bot-dev"
echo "  All services:         docker compose --profile navigation --profile manipulation --profile visualization up"
echo ""
echo "To stop all services:   docker compose down"
echo "To rebuild:            docker compose build"
echo ""
echo "Note: Make sure to run 'xhost +local:docker' if you encounter X11 permission issues"