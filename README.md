# RRT*

This project is a ROS-based implementation of [Quick-RRT*](https://www.sciencedirect.com/science/article/abs/pii/S0957417419300326) (proposed by Jeong et al.), a modified version of the well-known RRT* path planning algorithm. Compared to RRT*, Quick-RRT* generates a better initial solution by considering the concept of ancestry.

This implementation performs path planning for differential drive robots. Paths between points can be computed through different algorithms. The desired algorithm can be selected by simply editing the `config/params.yaml` file (the currently supported algorithms are: dubins path generation, [POSQ extend function](http://www.spencer.eu/papers/palmieriICAPS14.pdf) and cubic spline interpolation).

In this project, ROS is only needed in order to use _rviz_ as a ready-to-use GUI. Indeed, the search algorithm implementation does not depend on ROS.

Tested on Ubuntu 18.04 (ROS melodic), Ubuntu 20.04 (ROS noetic) and Ubuntu 22.04 (using Docker, give a look at the dedicated section for more details).

## Build

To build this project, the *tf2-geometry-msgs* package is needed. It can be installed through:

```
sudo apt install ros-${ROS_DISTRO}-tf2-geometry-msgs
```

Then you can simply clone this repository in your ROS workspace and build it through `catkin_make`.

To launch the demo, the *map server* and *rviz* are needed:

```
sudo apt install ros-${ROS_DISTRO}-map-server ros-${ROS_DISTRO}-rviz
```


Another option is to install the dependencies through rosdep:

```
rosdep install --from-paths <path/to/rrt/package>
```

## Demo

* Launch the demo through:
  ```
  roslaunch rrt rrt_demo.launch
  ```

  Then you can use the built-in rviz buttons `2D Pose Estimate` and `2D Nav Goal` to select starting point and goal, respectively. Path planning starts automatically once start and goal are set. The resulting path and the explored tree will be displayed on rviz.
* To load a different map, you can simply put the desired map inside the `maps/` folder and pass its name as argument of the roslaunch command:
  ```
  roslaunch rrt rrt_demo.launch map_name:=dummy_map
  ```
* You can play around with the most relevant parameters by simply editing the config file `config/params.yaml`!

## Docker

You can use the provided `Dockerfile` to launch the demo inside a container, without installing any dependencies.

To build the Docker image:
```
docker build -f Dockerfile --build-arg UID=$(id -u) --build-arg GID=$(id -g) --build-arg USERNAME=$(whoami) -t rrt-docker .
```
To run it:

```
docker run -it --rm --env DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix rrt-docker:latest
```

### Examples

---

**Dubins paths**

![alt text](gifs/dubins.gif)

**POSQ extend function**

![alt text](gifs/posq.gif)

**Cubic spline interpolation**

![alt text](gifs/spline.gif)
