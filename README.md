# vector3
Easy vector, quaternion library designed for rotations and sensor fusion filters in C++14.
It is also mobile device agnostic as mobile devices,such as iphones use y axis front and pitch is around X axis.
*
Example codes:
```c++
    using namespace vector3;
    
    auto v = Vector3::X;
    
    auto roll = rad(40.0);
    auto pitch = rad(40.0);
    auto yaw = rad(40.0);
    auto q = Quaternion(roll,pitch,yaw);
    
    q = Quaternion(roll,pitch,yaw,RotationOrder::rpy);
    
    q = q*Quaternion(roll,0,0);
    
    Vector3 to = v*q;
    
    Quaternion q1 = v.rotation(to);
    
    std::cout << "roll: " << deg(q.roll())<<", pitch: " << deg(q.pitch()) 
              << ", yaw: " << deg(q.yaw()) << endl;
    
    std::cout << "roll: " << deg(to.roll())<<", pitch: " << deg(to.pitch()) 
              << ", yaw: " << deg(to.yaw()) << endl;
```

by Maksim Piriyev @ https://dron.ee
