Copyright (c) Sensrad 2023-2024



## `arbe_raf`

This package encapsulates Arbe Robotics's reference driver, so it can be used from other ROS2 packages.



### Changes

* Currently using latest 1.8.5 files from Arbe GUI.

  

### Building

To build with ROS2 colcon:
```bash
colcon build --packages-select arbe_raf
```



### Using

This is a library package which can be used by adding the following in your CMakeList.txt and package.xml files.

CMakeList.txt:
`find_package(arbe_raf REQUIRED)`

package.xml:
`<depend>arbe_raf</depend>`

