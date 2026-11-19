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

COPY docker/zshrc /etc/zsh/user/.zshrc
COPY docker/starship.toml /etc/starship.toml
# The container runs as your user, so these must be readable by everyone.
RUN chmod 755 /etc/zsh/user && chmod 644 /etc/zsh/user/.zshrc /etc/starship.toml
ENV ZDOTDIR=/etc/zsh/user \
    STARSHIP_CONFIG=/etc/starship.toml \
    SHELL=/usr/bin/zsh
