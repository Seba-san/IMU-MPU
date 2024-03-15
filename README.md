# IMU MPU9150 ROS Package

Este paquete de ROS proporciona una interfaz para interactuar con el sensor MPU9150. Permite la recopilación y publicación de datos de aceleración, giroscopio y magnetómetro mediante el uso de ROS. Este repositorio también incluye un `Dockerfile` para facilitar la implementación y el entorno de ejecución en contenedores Docker.

## Requisitos

- Ubuntu 20.04
- ROS Noetic
- Python 3
- Acceso a un bus I2C (específicamente `/dev/i2c-1` en este caso)

## Instalación

Primero, asegúrate de tener ROS Noetic instalado en tu sistema. Para instalar ROS Noetic, puedes seguir las instrucciones oficiales en [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

Una vez instalado ROS, sigue estos pasos para instalar el paquete:

1. Clona este repositorio en el directorio `src` de tu espacio de trabajo de catkin:

```bash
cd ~/catkin_ws/src
git clone https://github.com/<usuario>/<repositorio>.git
```

2. Navega de regreso al directorio raíz de tu espacio de trabajo de catkin y compila:

```bash
cd ..
catkin_make
```

3. Asegúrate de que los scripts Python son ejecutables:

```bash
chmod +x ~/catkin_ws/src/<repositorio>/imu.py
chmod +x ~/catkin_ws/src/<repositorio>/ros_imu.py
```

## Uso

Para ejecutar el nodo de publicación de datos IMU, asegúrate de que tu entorno ROS esté configurado correctamente y luego ejecuta:

```bash
rosrun <nombre_del_paquete> ros_imu.py
```

Para construir y ejecutar el contenedor Docker, asegúrate de tener Docker instalado y luego ejecuta:

```bash
cd <directorio_del_repositorio>
./run.sh <nombre_de_imagen>
```

## Docker

El `Dockerfile` proporcionado en este repositorio permite ejecutar el paquete en un entorno aislado. Para construir la imagen Docker, usa el siguiente comando:

```bash
docker build -t <nombre_de_imagen> .
```

Luego, puedes ejecutar el contenedor utilizando el script `run.sh`.

## Contribuir

Las contribuciones son bienvenidas. Por favor, siente libre de forkear el repositorio y abrir un pull request con tus adiciones o cambios.

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Consulta el archivo `LICENSE` para obtener más detalles.
