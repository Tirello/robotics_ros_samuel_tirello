# Etapa 1: Imagem base com ROS jazzy
FROM ros:jazzy

# Evita que qualquer instalação peça uma interação com o usuário
ENV DEBIAN_FRONTEND=noninteractive

# Etapa 2: Pacotes de sistema
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    gedit \
    build-essential \
    python3 \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    nano \
  && rm -rf /var/lib/apt/lists/*

# Etapa 3: Pacotes ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Etapa 4: Configuração do ambiente de usuário
ARG USERNAME=host
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -eux; \
    # renomeia grupo 1000 para 'host' (se ainda não tiver esse nome)
    if [ "$(getent group ${USER_GID} | cut -d: -f1)" != "${USERNAME}" ]; then \
        groupmod -n "${USERNAME}" "$(getent group ${USER_GID} | cut -d: -f1)"; \
    fi; \
    # renomeia user 1000 para 'host' e move a home para /home/host
    if [ "$(getent passwd ${USER_UID} | cut -d: -f1)" != "${USERNAME}" ]; then \
        usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m "$(getent passwd ${USER_UID} | cut -d: -f1)"; \
    fi; \
    # sudoers e diretórios
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

COPY docker/config/bashrc /home/${USERNAME}/.bashrc

# Troca de usuário
USER ${USERNAME}
ENV HOME=/home/${USERNAME}

# Define qual é o diretório padrão de trabalho
WORKDIR /home/${USERNAME}/ros_ws

# Etapa 5: Comando padrão para abrir o terminal
CMD ["/bin/bash"]
