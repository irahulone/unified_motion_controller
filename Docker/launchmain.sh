set -e

# Define the Docker Compose file
DOCKER_COMPOSE_FILE="docker-compose.run.yml"

# Get host OS
OS_TYPE=$(uname)

echo "Detected OS: $OS_TYPE"

# Linux setup
if [[ "$OS_TYPE" == "Linux" ]]; then
  # Allow local user to access the X server
  xhost +local:root

  # Run Docker Compose with display environment
  DISPLAY_VAR=${DISPLAY:-:0}
  echo "Using DISPLAY=$DISPLAY_VAR"
  DISPLAY=$DISPLAY_VAR \
  docker compose -f $DOCKER_COMPOSE_FILE up --build

  # Revoke access after use
  xhost -local:root

# macOS setup
elif [[ "$OS_TYPE" == "Darwin" ]]; then
  echo "Setting up X11 forwarding on macOS..."

  # Make sure XQuartz is running
  if ! pgrep -x "XQuartz" > /dev/null; then
    echo "Starting XQuartz..."
    open -a XQuartz
    sleep 2
  fi

  # Allow connections from localhost
  xhost + 127.0.0.1

  # Get IP address of the host on macOS (host.docker.internal works in Docker Desktop)
  export DISPLAY=host.docker.internal:0

  docker compose -f $DOCKER_COMPOSE_FILE up --build

  # Cleanup
  xhost - 127.0.0.1

else
  echo "Unsupported OS: $OS_TYPE"
  exit 1
fi