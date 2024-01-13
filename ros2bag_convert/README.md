# ros2bag_convert

this Software permit to convert bag file to csv

## I. Installation

Download and install.

clone this repository

```
cd ros2_convert
python3 setup.py bdist_wheel
sudo pip install dist/ros2bag_convert-0.1.0-py3-none-any.whl
```

## II. Use

remember to source "install/setup.bash" on eufs directory for read the custom messages.
For run the conversion you have to use this command:
```
ros2bag-convert xxxx.db3
```
the result will be output to `xxx.db3` sibling directory.



