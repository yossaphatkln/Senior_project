# Senior Project

## Build

To build the project, run
```yaml
catkin build

```
or
```yaml
catkin_make

```
## Launch

Launch project
```yaml
roslaunch rocker_bogie main.launch

```
To enter SLAM mode
```yaml
roslaunch rocker_bogie main.launch mode:=slam

```
To enter navigation mode
```yaml
roslaunch rocker_bogie main.launch mode:=nav

```
## Test

To run runway test
```yaml
roslaunch rocker_bogie test.launch

```
To run rotate test
```yaml
roslaunch rocker_bogie test.launch mode:=rotate

```
