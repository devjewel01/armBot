docker run -it --rm ^
    --name armbot ^
    -e DISPLAY=host.docker.internal:0.0 ^
    -v "%cd%\src:/armBot/src:rw" ^
    -v "%cd%\rebuild.sh:/armBot/rebuild.sh" ^
    --network=host ^
    armbot:jazzy