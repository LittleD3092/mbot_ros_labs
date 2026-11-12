MBOT_IP ?= 67.194.34.59
IMAGE   := mbot-ros-labs:jazzy
CONTAINER ?= mbot-ros
MBOT_HOST ?= mbot@$(MBOT_IP)
ROS_ENV   := source /opt/ros/jazzy/setup.zsh && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

DOCKER_RUN := docker run -it --rm --net=host \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri \
  -e ROS_STATIC_PEERS=$(MBOT_IP) -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --user $$(id -u):$$(id -g) -e HOME=/tmp \
  -v $(CURDIR):/ws -w /ws

# Long-lived container that `make shell` can attach to.
DOCKER_RUN_NAMED := $(DOCKER_RUN) --name $(CONTAINER) $(IMAGE)
DOCKER_RUN       += $(IMAGE)

# zsh sources ROS and the workspace itself (docker/zshrc).
SHELL_INIT := exec zsh

.PHONY: rviz image build run shell clean lidar-on lidar-off

# Docker reuses cached layers, so this is quick when nothing changed.
image:
	docker build -t $(IMAGE) .

build: image
	$(DOCKER_RUN) bash -c 'colcon build --packages-select mbot_description'

run: image
	xhost +local: >/dev/null
	$(DOCKER_RUN_NAMED) bash -c '$(SHELL_INIT)'

# Open another terminal inside the running container (from `make run` or `make rviz`).
shell:
	@docker ps -q -f name=^$(CONTAINER)$$ | grep -q . || \
	  { echo "No '$(CONTAINER)' container running. Start one with 'make rviz' or 'make run'."; exit 1; }
	docker exec -it $(CONTAINER) bash -c '$(SHELL_INIT)'

rviz-mapping: build
	xhost +local: >/dev/null
	$(DOCKER_RUN_NAMED) bash -c 'source install/setup.bash && ros2 run rviz2 rviz2 -d mbot_slam/rviz/mapping.rviz'

rviz-localization: build
	xhost +local: >/dev/null
	$(DOCKER_RUN_NAMED) bash -c 'source install/setup.bash && ros2 run rviz2 rviz2 -d mbot_slam/rviz/localization.rviz'

rviz-slam: build
	xhost +local: >/dev/null
	$(DOCKER_RUN_NAMED) bash -c 'source install/setup.bash && ros2 run rviz2 rviz2 -d mbot_slam/rviz/slam.rviz'

clean:
	rm -rf build install log

lidar-off:
	ssh $(MBOT_HOST) '$(ROS_ENV) && ros2 service call /lidar_power std_srvs/srv/SetBool "{data: false}"'

lidar-on:
	ssh $(MBOT_HOST) '$(ROS_ENV) && ros2 service call /lidar_power std_srvs/srv/SetBool "{data: true}"'
