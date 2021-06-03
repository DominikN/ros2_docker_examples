# ros2_docker_examples

## Run without Docker

```bash
cd ros2_ws
colcon build

source /opt/ros/foxy/setup.bash
source install/setup.bash # full path might be: ~/ros2_docker_examples/ros2_ws/install/setup.bash

ros2 launch my_turtle_bringup turtlesim_demo.launch.py
```


## Run in a single container

**Please stay in `ros2_docker_examples/` directory while executing those commands:**

```
sudo chmod +x eg1/init-container.sh
sudo chmod +x eg1/ros_entrypoint.sh

docker build -t turtle_demo -f eg1/Dockerfile .

xhost local:root

docker run --rm -it \
--env DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
turtle_demo
```

## Run in two container (using `docker-compose`)

```bash
cd eg2
docker-compose up
```

## Run on two separate computers over VPN

tip: to access shell of running container:

```
docker exec -it <container_id> bash
```

```bash
cd eg3/dev1
docker-compose build
docker-compose up
```

```bash
cd eg3/dev2
docker-compose build
docker-compose up
```
