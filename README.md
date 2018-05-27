# vector3
Easy vector, quaternion library designed for rotations and sensor fusion filters in C++14.
It is also mobile device agnostic as mobile devices,such as iphones use y axis front and pitch is around X axis.

# quaternions
* quaternion from euler all the angles should be in radians
    ```c++
    auto roll = rad(40.0);
    auto pitch = rad(40.0);
    auto yaw = rad(40.0);
    auto q = Quaternion(roll,pitch,yaw);
    ``` 
 * change euler order in constructor of specific quaternion,ex: first roll then pitch then yaw
    ```c++
    auto roll = rad(40.0);
    auto pitch = rad(40.0);
    auto yaw = rad(40.0);
    auto q = Quaternion(roll,pitch,yaw,RotationOrder::rpy);
    ``` 
 * to euler as roll,pitch,yaw
    ```c++
    auto roll =  Quaternion(roll,pitch,yaw).roll();
    auto pitch =  Quaternion(roll,pitch,yaw).pitch();
    auto yaw =  Quaternion(roll,pitch,yaw).yaw();
    ``` 
* to euler as Vector3
    ```c++
    Vector3 euler =  Quaternion(roll,pitch,yaw).euler();
    ``` 
* concatanating rotations is as simple as multiplying quaternions (q1,q2,q3,q4) in respective order
    ``` q1*q2*q3*q4 ``` 
* rotating vector3 is as simple as multiplying with quaternion
    ``` v1*q1 ```
* reverse rotation is negating
    ``` -q1```
* rotation between two quaternions
    ``` -q1*q2 ```
* slerp is as simple as multiplying quaternion with scalar ex half angle rotation of q1 is
    ``` 0.5*q1 ``` 
* mid rotation between two quaternions is
    ``` q1*(0.5*(-q1*q2)) ```
    
# vectors
* constructor and constant static variables
    ``` Vector3 v(0.5,0.7,0.3);```
* from constant static variables
    ``` Vector3 v = Vector3::X; ```
* c++11 version
    ``` auto v = Vector3::X; ```
* to Vector2
    ```c++
    Vector2 xy = Vector3::X.xy(); 
    auto xz = Vector3::X.xz(); 
    auto yz = Vector3::X.yz(); 
    ```
* negating
    ``` auto negx = -Vector3::X; ```
* arithmetic operations
    ``` auto v = (0.3*(Vector3::X+Vector3::Y)).normalized();```
* multiplying with scalar all xyz
    ``` auto v2 = v1*0.5```
* multiplying with vector
    ```c++
        Vector3 coef(0.5,0.3,0.1);
        auto v = coef*Vector3::X;
     ```
* angle between two  vectors
    ```c++
        auto angleInRad = Vector3::X.angle(Vector3::Y);
        auto angleInDegree = deg(angleInRad);
     ```
* using in filtering
     ```c++
        auto beta = 0.1;
        auto result = Vector3::X*beta + Vector3::Y*(1-beta);
     ```
* rotation quaternion between two vectors
     ``` Quaternion q = Vector3::X.rotation(Vector3::Y); ```
* rotation quaternion around specific axis (not that there are many axis that can yield to same roation result)
     ``` Quaternion q = Vector3::X.rotation(Vector3::Y,-Vector3::Z); ```
* to array 
     ```vector<scalar> v = Vector3::X;```
* from array or initializer_list
     ```Vector3 v = {0.2,0.30,0.5};```
* from vector iterator
     ```c++
        vector<scalar> vec = {0.2,0.30,0.5,0.45};
        Vector3 v = vec.begin()+1;
     ```
# vector3 arithmetic helper functions
* degree to radian and vice versa conversion
     ```c++
        inline scalar rad(scalar deg);
        inline scalar deg(scalar rad);

    ```

* max,min,pow,abs
    ```c+++
        inline Vector3 pow(const Vector3& v,scalar p);
        inline Vector3 abs(const Vector3& v);
        inline Vector3 max(const Vector3& v1,const Vector3& v2);
        inline Vector3 min(const Vector3& v1,const Vector3& v2);
    ```

# mobile device vs others
* mobile devices definition of roll,pitch and reference frame XYZ is different than other engineering areas,
  define ```MOBILE``` if your application is for mobile or mobile data
    ``` #define MOBILE ````
* it is also automatically defined in header as
    ```c+++
    #ifdef TARGET_OS_IOS  or TARGET_IPHONE_SIMULATOR  or __ANDROID__             
        #define MOBILE
    #endif
    ```
# sensors
* sensor type
    ```c++
    enum SensorType{
        none = 0,accelerometer,gyroscope,magnetometer,coreMotion,position,velocity,userAcceleration,pixels
    };
    ```
* sensor data
    ```c++
    class SensorData{
        public:
        double time = 0; // in secs
        SensorType type = SensorType::none;
        Vector3 v = Vector3::Zero;
     };
    ```
# file operations
File read/write operations are simplified
* reading vector3 list from file
    ```c++
        vector<Vector3> rtn;
        ifstream(path)>>rtn;
    ```
* reading SensorData list from file
    ```c++
        vector<SensorData> rtn;
        ifstream(path)>>rtn;
    ```
* writing vector3 list to file
    ```c++
        vector<Vector3> rtn = {Vector3::X,Vector3::Y,Vector3::Z};
        ofstream(path)<<rtn<<endl;
    ```
* writing SensorData list to file
    ```c++
        vector<SensorData> rtn;
        ofstream(path)<<rtn<<endl;
    ```
    
# n dimensional ternary search
* use n dimensional search when local minima is equal to global minima, otherwise it probably will find one of local minimas
    ```c++
    vector<scalar> ternary(vector<scalar> start, vector<scalar> end, function<scalar(vector<scalar>)> eval);
    ```
* exmaple code for finding  calibration parameters of magnetometer
     ```c++
     void findMagBiasCollectively(const vector<SensorData>& list){
        if(list.size()==0) return;
        auto mlist = SensorData::filter(list, SensorType::magnetometer);
        Vector3 mx = mlist[0].v,mn=mlist[0].v;
        for(auto i:mlist){
            mx = vector3::max(i.v,mx);
            mn = vector3::min(i.v,mn);
        }

        auto biasList =
        vector3::ternary( mn.array()   , mx , [&](vector<scalar> p){
            Vector3 v(p);
            auto r = 0.0;
            for(auto i : mlist){
                r += ((i.v - v)).length();
            }
            r /= mlist.size();
            auto error = 0.0;
            for(auto i : mlist){
                auto t = ((i.v - v)).length() - r;
                error += t*t;
            }
            return error;
        });
        magBias = biasList;
    }
    ```

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
