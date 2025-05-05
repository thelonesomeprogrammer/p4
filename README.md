# P4 Semester Project
---

## Table of Contents
1. [Getting Started](#getting-started)
2. [Project Structure](#project-structure)
3. [Contributing](#contributing)

---

## Getting Started

Follow these steps to set up the project on your local machine:

### Prerequisites (for ground station)

#### all local
Ensure you have the following installed:
- ROS 2 (jazzy)
- Rust (via [Rustup](https://rustup.rs/))
- Trunk (for Rust components)
- wasm-target (for WebAssembly)
- Cargo-ament (for building Rust packages)
- yarn (for JavaScript components)

#### Docker
- Docker (for containerized environment)
- Docker buildx (for building images)



#### Python Dependencies
Install the required Python packages:
- cflib

### Clone the Repository

```bash
git clone https://github.com/thelonesomeprogrammer/p4.git
cd p4
```

### Running the Application with Docker

Build and run the containerized environment:

```bash
docker buildx build -t p4-station .
docker run -ti --rm -v ./:/workdir --privileged --network host p4-station
```
with the container running, you can run the following command to start the ground station:
```bash
colcon build
./start_station_tmux.sh
```
Visit the application in your browser at `http://localhost:3000`.

### Running the Application Locally
with all dependencies installed, you can run the application locally without Docker:

frist build the project:
```bash
colcon build
```
#### manual run
run the application:
terminal 1 (vicon bridge): 
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch launch/aal_segments.launch.py
```
terminal 2 (ground station):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run vicon-crazy vicon_stream
```
terminal 3 (web api):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run cf_rust_gui web
```
terminal 4 (frontend):
```bash
cd src/gui/react/dashboard/
yarn install
yarn run dev
```

#### lanch file
you can also run the application using a launch file:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch launch/all_vicon_gui.py
```
---

## Project Structure

```
p4/
├── app/             # apps til cf applayer oot er den primære
├── cache/           # cache til cf param of log 
├── launch/          # ros2 launch scripts
├── src/             # ros2 pakker 
├── tests/           # tests scripts fx til motor transforfunctoins 
├── .git*/           # git configurations filer 
├── Dockerfile       # Dockerfile
└── README.md        # Project documentation
```

---

## contributing
hvis du vil bidrage til projektet, skal du følge disse trin, hvis du er med i projektet:

1. lav en ny branch: `git checkout -b feature-branch`
2. Commit dine ændringer: `git commit -m "Add your message"`
3. Push til github: `git push origin feature-branch`
4. fortæl Marrinus om dine ændringer

