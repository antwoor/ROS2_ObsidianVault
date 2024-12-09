# Basic principles
## [[Nodes]] communicate via [[Messages|messages]] that are being transmitted through [[Topics|topics]]
![[Topic-MultiplePublisherandMultipleSubscriber.gif]]

# Structure of workspace

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```
## [[Creating packages| package creation]]

# [[Simulation of robots|Robot simulating]]
Depends on [[Simulation of robots#URDF|URDF]] files - **Universal Robot Description Format**
