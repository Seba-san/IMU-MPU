#!/bin/bash
number="${1:-}"
docker kill contenedor"$number"
docker rm contenedor"$number"