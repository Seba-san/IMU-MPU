#!/bin/bash
number="${1:-}"
xhost +local:*
docker exec -e "TERM=xterm-color" -it contenedor"$number" bash