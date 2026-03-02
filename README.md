Usage Instructions

 After you    
 ```git clone   https://github.com/5dbo/xarm_shape_tracer.git    ```
 
# Build
```
cd ~/dev_ws  

colcon build --packages-select xarm_shape_tracer  
source install/setup.bash  
```
# Launch  
```
ros2 launch xarm_shape_tracer trace_shapes.launch.py
```
# Multiple Shape Types

Multiple Shape Types: Lines, rectangles, circles, and polygons

# Adding new shape types:
extend ShapeType enum and implement generation methods


