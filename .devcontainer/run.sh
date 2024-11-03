# If not working, first do: sudo rm -rf /tmp/.docker.xauth

#!/bin/bash

# Allow local root user to access the X server
xhost local:root

# Set up the X authority file
XAUTH=/tmp/.docker.xauth

# Run the Docker container with the specified environment variables and volume mounts
docker run -it \
    --name=armbot_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    armbot_image \
    bash

echo "Done."
