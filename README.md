# ros2_docker_examples

This repo shows different ways to deal with ROS 2 node interconnectivity depending whether you:

- use them on a single machine or on multiple machines
- using with or without docker
- connecting over LAN or WAN

## [Eg. 0] Run without Docker

![without docker](docs/fig1-system-architecture.png)

```bash
cd ros2_ws
colcon build

source /opt/ros/foxy/setup.bash
source install/setup.bash # full path might be: ~/ros2_docker_examples/ros2_ws/install/setup.bash

ros2 launch my_turtle_bringup turtlesim_demo.launch.py
```


## [Eg. 1] Run in a single container

![without docker](docs/fig2-one-container.png)

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

## [Eg. 2] Run in two containers (using `docker-compose`)

![without docker](docs/fig3-two-containers.png)

```bash
cd eg2
docker-compose up
```

## [Eg. 3] PROBLEM: Runing on two computers in different networks

![without docker](docs/fig4-two-containers-two-networks.png)

Because two ROS 2 devices are in different networks, DDS can not perform auto-discovery.

Also devices can not reach each other because they do not have nor public nor static IP addresses and are behind Wi-Fi router NAT. 

## [Eg. 3] SOLUTION 1: Connect ROS 2 machines using VPN

To enable communication between remote containers we need to do two things:

- install & configure Husarnet VPN client
- create a custom DDS confiugration file 

Ready to use example is available in `eg3/` folder. There are two separate subfolders with a `docker-compose.yml` file to be launched on two separate devices from different networks.

### connect containers to the same VPN network

At first modify `eg3/dev1/.env` and `eg3/dev2/.env` files by providing the same Husarnet network Join Code there. To find your joincode go to:

-> app.husarnet.com 

-> choosen network (or create a new) 

-> click **[Add element]** button 

-> copy the join code to a clipboard.

Then go to `eg3/dev1/` folder on first machine, and `eg3/dev2` folder on second machine and execute:

```bash
docker-compose up
```

You will se the following output log:

```bash
dominik@legion-y540:~/tech/ros2_docker_examples/eg3/dev1$ docker-compose up
Starting dev1_turtle_controller_1 ... done
Attaching to dev1_turtle_controller_1
turtle_controller_1  | 
turtle_controller_1  | ⏳ [1/2] Initializing Husarnet Client:
turtle_controller_1  | waiting...
turtle_controller_1  | waiting...
turtle_controller_1  | waiting...
turtle_controller_1  | success
turtle_controller_1  | 
turtle_controller_1  | 🔥 [2/2] Connecting to Husarnet network as "turtle-controller":
turtle_controller_1  | [2216368] joining...
turtle_controller_1  | [2218369] joining...
turtle_controller_1  | done
turtle_controller_1  | 
turtle_controller_1  | *******************************************
turtle_controller_1  | 💡 Tip
turtle_controller_1  | IPv6 addr of this container is: fc94:a2cd:168a:1c7b:a135:e22f:e172:892f
turtle_controller_1  | *******************************************
```

containing a IPv6 address of the device `fc94:a2cd:168a:1c7b:a135:e22f:e172:892f`. You will use that address to configure a Cyclone DDS in the next step.

You can shutdown container now by **[ctrl + c]**

### Configure a cyclone DDS

In `cyclonedds.xml` file we need to specify a Husarnet VPN IPv6 address of all peers. So in `eg3/dev1/cyclonedds.xml` file you need to specify IPv6 addres of `dev1` device:

```xml
<Peers>
    <Peer address="[fc94:c37a:c18e:bc90:9b0b:7144:f212:9743]"/>
</Peers>
```

Similarly modify `eg3/dev2/cyclonedds.xml` by specifying Husarnet IPv6 addr of `dev1` there.

### Run the containers:

Now go to `eg3/dev1/` and `eg3/dev2/` folders on two machines and run:

```
cd eg3/dev2
docker-compose build
docker-compose up
```

Congrats! You have everything up and running.

![without docker](docs/screenshot.png)


## [Eg. 3] SOLUTION 2: Connect container on your laptop with turtlesim in the ROSject

![without docker](docs/fig5-sollution.png)


### Part to do on your laptop

```bash
cd eg3/dev1
docker-compose build
docker-compose up
```

and copy the container's Husarnet IPv6 address from an output log (or from a app.husarnet.com)

### Part to do in the ROSject

Create a new ROSject and run:

#### Terminal 1

```bash
sudo husarnet daemon
```

#### Terminal 2

##### 1. Connect your ROSject to Husarnet network:

```bash
sudo husarnet join <joincode> rosject1
```

Copy your ROSject's Husarnet IPv6 address to this part of `eg3/dev1/cyclonedds.xml` file:

```xml
<Peers>
    <Peer address="[fc94:c37a:c18e:bc90:9b0b:7144:f212:9743]"/>
</Peers>
```

##### 2. Install and configure Cyclone DDS

```
sudo apt update
sudo apt install ros-foxy-rmw-cyclonedds-cpp
cd ~
touch cyclonedds.xml
```

Copy the following XML to the `cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
            <Transport>udp6</Transport>
        </General>      
        <Discovery>
            <Peers>
                <Peer address="[fc94:c37a:c18e:bc90:9b0b:7144:f212:9743]"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>severe</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

In this line place the IPv6 address of the running container from your laptop.

```xml
<Peer address="[fc94:c37a:c18e:bc90:9b0b:7144:f212:9743]"/>
```

Save file and execute in the terminal:

```bash
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/user/ros2_ws/src/cyclonedds/cyclonedds.xml
```

##### 3. Run the turtlesim node

```bash
ros2 run turtlesim turtlesim_node
```

and on your laptop start the container:

```bash
cd eg3/dev1
docker-compose build
docker-compose run
```

In the ROSject (after you click `[Graphical tools]` button ) you will see:

![without docker](docs/screenshot.png)