FROM ros:noetic-ros-core

ARG USERNAME
ARG UID
ARG GID

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt upgrade -y && \
    apt install -y build-essential ros-noetic-rviz ros-noetic-map-server ros-noetic-tf2-geometry-msgs && \
    addgroup --gid $GID $USERNAME && \
    adduser --uid $UID --gid $GID --disabled-password $USERNAME

USER $USERNAME

COPY --chown=$UID:$GID . /home/$USERNAME/rrt

WORKDIR /home/$USERNAME/rrt

RUN source /opt/ros/noetic/setup.bash && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=Off -S . -B build/ && \
    cmake --build build/ && \
    rm -rf src/ include/ CMakeLists.txt

CMD source /opt/ros/noetic/setup.bash && \
    source build/devel/setup.bash && \
    # Disable hardware acceleration before running rviz
    export LIBGL_ALWAYS_SOFTWARE=1 && \
    roslaunch rrt rrt_demo.launch