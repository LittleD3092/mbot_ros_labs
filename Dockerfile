FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-msgs ros-jazzy-nav2-map-server \
      zsh git curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Installed system-wide: the container runs as your host user with HOME=/tmp.
RUN git clone --depth 1 https://github.com/ohmyzsh/ohmyzsh.git /opt/oh-my-zsh \
 && git clone --depth 1 https://github.com/zsh-users/zsh-autosuggestions \
      /opt/oh-my-zsh/custom/plugins/zsh-autosuggestions \
 && git clone --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git \
      /opt/oh-my-zsh/custom/plugins/zsh-syntax-highlighting

RUN curl -fsSL https://starship.rs/install.sh | sh -s -- -y -b /usr/local/bin

# Robot-side message package, normally provided by mbot_ws on the mbot.
# Built here as an underlay so the laptop needs no other repo. docker/ has a
# COLCON_IGNORE, so a colcon build on the robot never picks up this copy.
# Keep docker/mbot_interfaces in sync with mbot_ws/src/mbot_interfaces.
COPY docker/mbot_interfaces /opt/mbot_deps/src/mbot_interfaces
RUN bash -c 'source /opt/ros/jazzy/setup.bash \
      && cd /opt/mbot_deps && colcon build && rm -rf build log' \
 && sed -i 's|^exec "\$@"|source /opt/mbot_deps/install/setup.bash --\nexec "$@"|' /ros_entrypoint.sh

COPY docker/zshrc /etc/zsh/user/.zshrc
COPY docker/starship.toml /etc/starship.toml
# The container runs as your user, so these must be readable by everyone.
RUN chmod 755 /etc/zsh/user && chmod 644 /etc/zsh/user/.zshrc /etc/starship.toml
ENV ZDOTDIR=/etc/zsh/user \
    STARSHIP_CONFIG=/etc/starship.toml \
    SHELL=/usr/bin/zsh
