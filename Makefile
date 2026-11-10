MBOT_IP ?= 67.194.34.59
IMAGE   := osrf/ros:jazzy-desktop
MBOT_HOST ?= mbot@$(MBOT_IP)
ROS_ENV   := source /opt/ros/jazzy/setup.zsh && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

DOCKER_RUN := docker run -it --rm --net=host \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri \
  -e ROS_STATIC_PEERS=$(MBOT_IP) -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --user $$(id -u):$$(id -g) -e HOME=/tmp \
  -v $(CURDIR):/ws -w /ws $(IMAGE)

.PHONY: rviz build clean lidar-on lidar-off

build:
	$(DOCKER_RUN) bash -c 'colcon build --packages-select mbot_description'

run:
	$(DOCKER_RUN) bash

rviz: build
	xhost +local: >/dev/null
	$(DOCKER_RUN) bash -c 'source install/setup.bash && rviz2 -d src/mbot_bringup/rviz/mbot_viz.rviz'

clean:
	rm -rf build install log

lidar-off:
	ssh $(MBOT_HOST) '$(ROS_ENV) && ros2 service call /lidar_power std_srvs/srv/SetBool "{data: false}"'

lidar-on:
	ssh $(MBOT_HOST) '$(ROS_ENV) && ros2 service call /lidar_power std_srvs/srv/SetBool "{data: true}"'
