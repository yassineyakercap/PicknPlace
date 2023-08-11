docker run -it --rm \
    --privileged \
    -e DISPLAY=$DISPLAY \
    --device /dev/dri/  \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp:/tmp \
    -v /dev/shm:/dev/shm \
    --name picknplace_demo \
    $1 $2
