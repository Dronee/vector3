//
//  vector3.h
//  vector3
//
//  Created by Maksim Piriyev on 4/16/18.
//  Copyright © 2018 Dronee. All rights reserved.
//

#pragma once


#include <cmath>
#include <vector>


namespace vector3{
    using namespace std;
    
typedef float scalar;

//#define MOBILE
#ifdef TARGET_OS_IOS  or TARGET_IPHONE_SIMULATOR  or __ANDROID__              // <--- Change this with a proper definition
    #define MOBILE
#endif

#define eps (1e-6)
inline scalar rad(scalar deg){ return deg*M_PI/180;}
inline scalar deg(scalar rad){ return rad*180/M_PI;}

enum RotationOrder{
    ypr, yrp, pry, pyr, rpy, ryp
};


struct Vector2 {
public:
    const static Vector2 Zero;
    const static Vector2 X;
    const static Vector2 Y;
    
    scalar x=0,y=0;
    Vector2(scalar x,scalar y):x(x),y(y){ }
    Vector2 inverse(){ return -(*this); }
    scalar sum(){return std::abs(x)+std::abs(y);}
    scalar length()const{return sqrt(x*x+y*y);}
    scalar lengthSquared()const{return x*x+y*y;}
    vector<scalar> array()const{ return {x,y};}
    void array(vector<scalar> v){  x = v[0]; y =v[1]; }
    
    scalar dot(const Vector2& v)  {
        return x * v.x + y * v.y;
    }
    
    scalar cross(const Vector2& v) {
        return x * v.y - y * v.x;
    }
    
    Vector2 normalized()const {
        auto lengthSquared = this->lengthSquared();
        if (lengthSquared != 0 || lengthSquared != 1) {
            return *this;
        }
        return (*this) / sqrt(lengthSquared);
    }
    
    Vector2 rotated(scalar radians) {
        auto cs = cos(radians);
        auto sn = sin(radians);
        return Vector2(x * cs - y * sn, x * sn + y * cs);
    }
    
    Vector2 rotated(scalar radians, Vector2 pivot) {
        return ((*this) - pivot).rotated(radians) + pivot;
    }
    

    
    scalar angle(Vector2 with)  {
        if ( (*this) == with) {
            return 0;
        }
        return atan2(y, x) - atan2(with.y, with.x);
    }
    
    Vector2 interpolated(Vector2 with, scalar t) {
        return (*this) + (with - (*this)) * t;
    }
    Vector2 operator -()const { return Vector2(-x,-y);}
    Vector2 operator *(scalar v)const { return Vector2(x*v,y*v);}
    Vector2 operator /(scalar v)const { return Vector2(x/v,y/v);}
    friend Vector2 operator +(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x+v2.x,v1.y+v2.y); }
    friend Vector2 operator -(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x-v2.x,v1.y-v2.y); }
    friend Vector2 operator /(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x/v2.x,v1.y/v2.y); }
    friend Vector2 operator *(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x*v2.x,v1.y*v2.y); }
    friend bool operator ==(const Vector2& v1,const Vector2& v2){ return  std::abs(v1.x- v2.x) < eps && std::abs(v1.y - v2.y) < eps; }
    friend bool operator !=(const Vector2& v1,const Vector2& v2){ return  std::abs(v1.x- v2.x) > eps || std::abs(v1.y - v2.y) > eps; }
};


inline Vector2 pow2(const Vector2 v){
    return Vector2(v.x*v.x,v.y*v.y);
}
inline Vector2 abs(const Vector2 v) {
    return Vector2(std::abs(v.x),std::abs(v.y));
}
inline Vector2 max(const Vector2 v1,const Vector2 v2){
    return Vector2(std::max(v1.x,v2.x),std::max(v1.y,v2.y));
}
inline Vector2 min(const Vector2 v1,const Vector2 v2){
    return Vector2(std::min(v1.x,v2.x),std::min(v1.y,v2.y));
}

class Quaternion;

struct Vector3 {
public:
    const static Vector3 Zero,Head,X,Y,Z;
    scalar x=0,y=0,z=0;
    Vector3(scalar x,scalar y,scalar z):x(x),y(y),z(z){ }
    //int hash(){ return 0;}
    Vector3 inverse()const{ return -(*this); }
    scalar sum()const{return std::abs(x)+std::abs(y)+std::abs(z);}
    scalar length()const{return sqrt(x*x+y*y+z*z);}
    scalar lengthSquared()const{return x*x+y*y+z*z;}
    vector<scalar> array(){ return {x,y,z};}
    void array(vector<scalar> v){  x = v[0]; y =v[1]; z=v[2];}
    Vector2 xy(){return Vector2(x,y); }
    Vector2 xz(){return Vector2(x,z); }
    Vector2 yz(){return Vector2(y,z); }
    
    bool isSameDirection(const Vector3& v){
        auto d = dot(v);
        return  std::abs(d-1) < eps;
    }
    bool isOpposite(const Vector3& v){
        auto d = dot(v);
        return  std::abs(d+1) < eps;
    }
    scalar dot(const Vector3& v)const{
        return x * v.x + y * v.y + z * v.z;
    }
    
    Vector3 cross(const Vector3& v)const  {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    Vector3 normalized() const{
        auto ls = lengthSquared();
        if(std::abs(ls) < eps || std::abs(ls - 1) < eps ) {
            return (*this);
        }
        return (*this) / sqrt(ls);
    }

    void cutoff(Vector3& v){
        if(std::abs(x) < v.x) { x = 0;}
        if(std::abs(y) < v.y) { y = 0;}
        if(std::abs(z) < v.z) { z = 0;}
    }
    

    Vector3 interpolated(Vector3& to,scalar by){
        return (*this) + (to - (*this)) * by;
    }

    scalar pitch();
    
    scalar roll();
    
    scalar yaw() ;
    
    Quaternion rotation(const Vector3& to) const;
   
    Quaternion rotation(const Vector3& to,const Vector3& axis) ;
    Vector3 rotated(scalar angle,const Vector3& axis) ;
    
    Vector3 operator -()const{ return Vector3(-x,-y,-z);}
    friend Vector3 operator +(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x+v2.x,v1.y+v2.y,v1.z+v2.z); }
    friend Vector3 operator -(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x-v2.x,v1.y-v2.y,v1.z-v2.z); }
    friend Vector3 operator /(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x/v2.x,v1.y/v2.y, v1.z/v2.z); }
    friend Vector3 operator *(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x*v2.x , v1.y*v2.y , v1.z*v2.z); }
    friend Vector3 operator /(const Vector3& v1,const scalar v){ return  Vector3(v1.x/v,v1.y/v, v1.z/v); }
    friend Vector3 operator *(const Vector3& v1,const scalar v){ return  Vector3(v1.x*v , v1.y*v , v1.z*v); }
    friend Vector3 operator *(const scalar v,const Vector3& v1){ return  Vector3(v1.x*v , v1.y*v , v1.z*v); }
    friend bool operator ==(const Vector3& v1,const Vector3& v2){ return  std::abs(v1.x- v2.x) < eps && std::abs(v1.y - v2.y) < eps && std::abs(v1.z - v2.z) < eps; }
    friend bool operator !=(const Vector3& v1,const Vector3& v2){ return  std::abs(v1.x- v2.x) > eps || std::abs(v1.y - v2.y) > eps || std::abs(v1.z - v2.z) > eps; }

};


inline Vector3 pow2(const Vector3& v){
    return Vector3(v.x*v.x,v.y*v.y,v.z*v.z);
}
inline Vector3 pow(const Vector3& v,scalar p){
    return Vector3(std::pow(v.x,p),std::pow(v.y,p),std::pow(v.z,p));
}
inline Vector3 abs(const Vector3& v) {
    return Vector3(std::abs(v.x),std::abs(v.y),std::abs(v.z));
}
inline Vector3 max(const Vector3& v1,const Vector3& v2){
    return Vector3(std::max(v1.x,v2.x),std::max(v1.y,v2.y),std::max(v1.z,v2.z));
}
inline Vector3 min(const Vector3& v1,const Vector3& v2){
    return Vector3(std::min(v1.x,v2.x),std::min(v1.y,v2.y),std::min(v1.z,v2.z));
}



class Quaternion {
    inline scalar _roll() {
        //roll in regular
        auto t0 = 2 * (w * x + y * z);
        auto t1 = 1 - 2 * (x * x + y*y);
        return atan2(t0, t1);
        
//        auto t0 = 2 * (w * x + y * z);
//        auto t1 = w*w + z*z - x*x * y*y;// - 2 * (x * x + y*y);
//        return atan2(t0, t1);
        //return atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
    }
    
    inline scalar _yaw() {
        auto t3 = 2 * (w * z + x * y);
        auto t4 = 1 - 2 * (y*y + z * z);
        return atan2(t3, t4);
        // return asin(-2 * (x * z - w * y))
    }
    
    inline scalar _pitch() {
        auto t2 = 2.0 * (w * y - z * x);
        t2 = ((t2 > 1.0) ? 1.0 : t2);
        t2 = ((t2 < -1.0) ? -1.0 : t2);
        
        //        auto t0 = 2 * (w * x + y * z);
        //        auto t1 = w*w + z*z  - y*y - x*x;
        //   return atan2(t0,t1);
        return asin(t2); // pitch in regular
        //return atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
    }
public:
    const static Quaternion Zero,Identity;
    scalar x=0,y=0,z=0,w=1;
  
    Quaternion(scalar x,scalar y,scalar z,scalar w):x(x),y(y),z(z),w(w){ }
    Quaternion(const Vector3& axis,scalar angle){
        auto r = angle * 0.5;
        auto scale = sin(r);
        auto a = axis * scale;
        x = a.x;
        y = a.y;
        z = a.z;
        w = cos(r);
     }
#ifdef MOBILE
    Quaternion(Vector3 gyro,RotationOrder order = RotationOrder::ypr):Quaternion(gyro.y,gyro.x,-gyro.z,order) {}
#else
    Quaternion(Vector3 gyro,RotationOrder order = RotationOrder::ypr):Quaternion(gyro.x,gyro.y,gyro.z,order) {}
#endif
    Quaternion(scalar roll,scalar pitch,scalar yaw,RotationOrder order = RotationOrder::ypr) { // default ypr
#ifdef MOBILE
        auto quatPitch = Quaternion(Vector3::X, pitch);
        auto quatYaw = Quaternion(-Vector3::Z, yaw);
        auto quatRoll = Quaternion(Vector3::Y, roll);
#else
        auto quatPitch = Quaternion(Vector3::Y, pitch);
        auto quatYaw = Quaternion(Vector3::Z, yaw);
        auto quatRoll = Quaternion(Vector3::X, roll);
#endif

        switch(order) {
        case RotationOrder::ypr:  (*this) = quatYaw * quatPitch * quatRoll; break;
        case RotationOrder::yrp:  (*this) = quatYaw * quatRoll * quatPitch; break;
        case RotationOrder::pry:  (*this) = quatPitch * quatRoll * quatYaw; break;
        case RotationOrder::pyr:  (*this) = quatPitch * quatYaw * quatRoll; break;
        case RotationOrder::rpy:  (*this) = quatRoll * quatPitch * quatYaw; break;
        case RotationOrder::ryp:  (*this) = quatRoll * quatYaw * quatPitch; break;
        default: (*this) = quatYaw * quatPitch * quatRoll; break;
            
        }
    }
    Quaternion inverse(){ return -(*this); }
    scalar sum(){return std::abs(x)+std::abs(y)+std::abs(z)+std::abs(w);}
    scalar length(){return sqrt(x*x+y*y+z*z+w*w);}
    scalar lengthSquared(){return x*x+y*y+z*z+w*w;}
    vector<scalar> array(){ return {x,y,z,w};}
    void array(vector<scalar> v){  x = v[0]; y =v[1]; z=v[2]; w = v[3];}
    Vector3 xyz()const {return Vector3(x,y,z); }
    
    Quaternion normalized() {
        auto ls = lengthSquared();
        if(std::abs(ls) < eps || std::abs(ls - 1) < eps ) {
            return (*this);
        }
        return (*this) / sqrt(ls);
    }
    void normalize() {
        auto ls = lengthSquared();
        if(std::abs(ls) < eps || std::abs(ls - 1) < eps ) {   return; }
        (*this) = (*this) / sqrt(ls);
    }
    Vector3 axis(){return Vector3(x,y,z);}
    scalar angle(){return acos(w) * 2;}
    scalar roll() {
        #ifdef MOBILE
        return _pitch();
        #else
        return _roll();
        #endif
      }
    
    scalar yaw(){
#ifdef MOBILE
        return -_yaw();
#else
        return _yaw();
#endif
    }
    
    scalar pitch() {
#ifdef MOBILE
        return _roll();
#else
        return  _pitch();
#endif
    }

    scalar dot(const Quaternion& v)const{
        return x * v.x + y * v.y + z * v.z + w * v.w;
    }
    Quaternion lerp(const Quaternion& to, scalar p){
        scalar op = 1 - p;
        return Quaternion(op*x + p*to.x, op*y + p*to.y, op*z + p*to.z, op*w + p*to.w).normalized();
    }
    Quaternion slerp(const Quaternion& to,scalar p){
        scalar theta = acos(x*to.x + y*to.y + z*to.z + w*to.w);
        scalar sn = sin(theta);
        scalar a = sin((1-p)*theta) / sn;
        scalar b = sin(p*theta) / sn;
        return Quaternion(a*x + b*to.x,a*y + b*to.y,a*z + b*to.z,a*w + b*to.w).normalized();
    }
    
    friend Vector3 operator *(const Vector3& v,const Quaternion& q){
        auto qv = q.xyz();
        auto uv = qv.cross(v);
        auto uuv = qv.cross(uv);
        return v + (uv * 2 * q.w) + (uuv * 2);
    }
    friend Vector3 operator *(const Quaternion& q,const Vector3& v){
        return v*q;
    }

    Quaternion operator -()const{ return Quaternion(-x,-y,-z,w);}
    friend Quaternion operator +(const Quaternion& v1,const Quaternion& v2){ return  Quaternion(v1.x+v2.x,v1.y+v2.y,v1.z+v2.z); }
    friend Quaternion operator -(const Quaternion& v1,const Quaternion& v2){ return  Quaternion(v1.x-v2.x,v1.y-v2.y,v1.z-v2.z); }
    friend Quaternion operator /(const Quaternion& v1,const Quaternion& v2){ return  Quaternion(v1.x/v2.x,v1.y/v2.y,v1.z/v2.z); }
    friend Quaternion operator *(const Quaternion& lhs,const Quaternion& rhs){
        return  Quaternion(lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x,
                lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z);
              }
    friend void operator *=(Quaternion& lhs,const Quaternion& rhs){
        lhs = Quaternion(lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
                           lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z,
                           lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x,
                           lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z);
    }
    
    friend Quaternion operator /(const Quaternion& v1,const scalar v){ return  Quaternion(v1.x/v,v1.y/v,v1.z/v,v1.w/v); }
    friend Quaternion operator *(const Quaternion& v1,const scalar v){ return  Quaternion(v1.x*v,v1.y*v,v1.z*v,v1.w*v); }
    friend void operator *=(Quaternion& v1,const scalar v){ v1 = Quaternion(v1.x*v,v1.y*v,v1.z*v,v1.w*v); }
    friend bool operator ==(const Quaternion& v1,const Quaternion& v2){
        return  std::abs(v1.x- v2.x) < eps && std::abs(v1.y - v2.y) < eps && std::abs(v1.z - v2.z) < eps && std::abs(v1.w - v2.w) < eps;
    }
    friend bool operator !=(const Quaternion& v1,const Quaternion& v2){
        return  std::abs(v1.x- v2.x) > eps || std::abs(v1.y - v2.y) > eps || std::abs(v1.z - v2.z) > eps || std::abs(v1.w - v2.w) > eps;
    }
};

}
