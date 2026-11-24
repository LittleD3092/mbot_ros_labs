# Included from the main Makefile (include sim/sim.mk).
# ---------------------------------------------------------------------------
# Simulation: fake MBot on the laptop (see sim/README.md).
# Own DDS domain, local-only discovery and no static peer, so nothing started
# here can talk to the real robot. Uses the same container name, so
# `make shell` opens more terminals in it.
SIM_MAP ?= maze1
SIM_DOCKER := docker run -it --rm --net=host \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri \
  -e ROS_STATIC_PEERS= -e ROS_DOMAIN_ID=77 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --user $$(id -u):$$(id -g) -e HOME=/tmp \
  -v $(CURDIR):/ws -w /ws

.PHONY: sim-build sim sim-explore

sim-build: image
	$(SIM_DOCKER) $(IMAGE) bash -c 'colcon build --packages-up-to mbot_nav mbot_slam'

# Shell in the sim container; run nodes yourself (one per `make shell`).
sim: sim-build
	xhost +local: >/dev/null
	$(SIM_DOCKER) --name $(CONTAINER) $(IMAGE) bash -c '$(SHELL_INIT)'

# Everything at once: fake MBot + SLAM + planner + controller + explorer + RViz.
sim-explore: sim-build
	xhost +local: >/dev/null
	$(SIM_DOCKER) --name $(CONTAINER) $(IMAGE) bash -c 'source install/setup.bash && ros2 launch sim/explore_sim.launch.py map:=$(SIM_MAP)'
