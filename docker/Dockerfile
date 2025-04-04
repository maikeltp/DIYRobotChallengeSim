FROM ros:jazzy-ros-core-noble

# Arguments for user creation
# ARG USERNAME=devuser
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# update base system
RUN apt-get update && apt-get upgrade -y --no-install-recommends

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev \
    bash-completion \
    build-essential \
    python3-colcon-devtools \
    python3-colcon-package-selection \
    python3-colcon-ros \
    python3-pip \
	python3-venv \
    ros-dev-tools \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-limits \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joy-linux \
    ros-jazzy-joy-teleop \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2launch \
    ros-jazzy-rplidar-ros \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro

# install gazebo & rviz2 packages
RUN apt-get install -y --no-install-recommends \
    psmisc \
    ros-jazzy-grid-map-rviz-plugin \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-image-geometry \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-ros2controlcli \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rviz2

# Create a virtual environment
RUN python3 -m venv /home/devuser/venv

# Install pyserial pyvesc in the virtual environment
RUN /home/devuser/venv/bin/pip install pyserial pyvesc pythoncrc

# Create a non-root user
RUN groupadd --gid $USER_GID devuser || groupmod -n devuser $(getent group $USER_GID | cut -d: -f1) \
    && useradd --uid $USER_UID --gid $USER_GID -m devuser \
    || usermod -l devuser -m -d /home/devuser $(id -un $USER_UID) \
    && apt-get update \
    && apt-get install -y sudo \
    && echo devuser ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/devuser \
    && chmod 0440 /etc/sudoers.d/devuser

# Add user to dialout group to allow serial port access
RUN usermod -aG dialout devuser

# Set up development workspace
RUN mkdir -p /home/devuser/ros2_ws
RUN chown -R devuser:devuser /home/devuser/ros2_ws
WORKDIR /home/devuser/ros2_ws

# Set up ros 2 environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/devuser/.bashrc \
    && echo "[[ -d /home/devuser/ros2_ws/install ]] && source /home/devuser/ros2_ws/install/setup.sh" >> /home/devuser/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/devuser/.bashrc \
	&& echo "source /home/devuser/venv/bin/activate" >> /home/devuser/.bashrc \
	&& echo 'export PYTHONPATH=/home/devuser/venv/lib/python3.12/site-packages:$PYTHONPATH' >> /home/devuser/.bashrc

# Set up colcon build defaults
RUN mkdir -p /home/devuser/.colcon \
    && echo "{ \
    \"build\": { \
    \"symlink-install\": true, \
    \"event-handlers\": [\"console_direct+\"], \
    \"parallel-workers\" : 6 \
    } \
    }" > /home/devuser/.colcon/defaults.yaml


# Switch to non-root user
USER devuser

CMD ["/bin/bash"]
