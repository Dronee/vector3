//
//  vector3.h
//  vector3
//
//  Created by Maksim Piriyev on 4/16/18.
//  Copyright © 2018 Dronee. All rights reserved.
//

#pragma once


#include <iostream>
#include <cmath>
#include <vector>
#include <functional>

namespace vector3{
    using namespace std;
    
    typedef double scalar;
    
#define MOBILE
#ifdef TARGET_OS_IOS  or TARGET_IPHONE_SIMULATOR  or __ANDROID__              // <--- Change this with a proper definition
#define MOBILE
#endif
    
#define eps (1e-6)
    inline scalar rad(scalar deg){ return deg*M_PI/180;}
    inline scalar deg(scalar rad){ return rad*180/M_PI;}
    
    enum RotationOrder{
        ypr, yrp, pry, pyr, rpy, ryp
    };
    
    class Vector3;
    
    struct Vector2 {
    public:
        const static Vector2 Head;
        const static Vector2 Zero;
        const static Vector2 One;
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
        
        
        scalar dot(const Vector2& v)const{
            return x * v.x + y * v.y;
        }
        scalar angle(Vector2 with = Vector2::Head)const  {
            if ( (*this) == with) {
                return 0;
            }
            return atan2(with.y, with.x) - atan2(y, x);
            // return atan2(y, x) - atan2(with.y, with.x);
        }
        
        Vector2 interpolated(Vector2 with, scalar t) {
            return (*this) + (with - (*this)) * t;
        }
        Vector2 operator -()const { return Vector2(-x,-y);}
        Vector2 operator *(scalar v)const { return Vector2(x*v,y*v);}
        Vector2 operator /(scalar v)const { return Vector2(x/v,y/v);}
        friend Vector2 operator +(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x+v2.x,v1.y+v2.y); }
        friend Vector2 operator -(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x-v2.x,v1.y-v2.y); }
        friend Vector2 operator -(const scalar p,const Vector2& v2){ return  Vector2(p-v2.x,p-v2.y); }
        friend Vector2 operator /(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x/v2.x,v1.y/v2.y); }
        friend Vector2 operator *(const Vector2& v1,const Vector2& v2){ return  Vector2(v1.x*v2.x,v1.y*v2.y); }
        friend bool operator ==(const Vector2& v1,const Vector2& v2){ return  std::abs(v1.x- v2.x) < eps && std::abs(v1.y - v2.y) < eps; }
        friend bool operator !=(const Vector2& v1,const Vector2& v2){ return  std::abs(v1.x- v2.x) > eps || std::abs(v1.y - v2.y) > eps; }
        
        operator Vector3()const;
        operator vector<scalar>()const{ return array();}
        Vector2& operator =(const vector<scalar>& v){
            x = v[0]; y = v[1]; ;
            return *this;
        }
        Vector2& operator =(scalar v){  x = v; y = v;
            return *this;
        }
        
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
        const static Vector3 Zero,Head,X,Y,Z,One,Max,Min;
        scalar x=0,y=0,z=0;
        Vector3(){ }
        Vector3(scalar x,scalar y,scalar z):x(x),y(y),z(z){ }
        Vector3(const vector<scalar>& v):x(v[0]),y(v[1]),z(v[2]){ }
        Vector3(const vector<scalar>::iterator& it):x(*it),y(*(it+1)),z(*(it+2)){ }
        //int hash(){ return 0;}
        Vector3 inverse()const{ return -(*this); }
        scalar sum()const{return std::abs(x)+std::abs(y)+std::abs(z);}
        scalar length()const{return sqrt(x*x+y*y+z*z);}
        scalar lengthSquared()const{return x*x+y*y+z*z;}
        vector<scalar> array()const{ return {x,y,z};   }
        void array(vector<scalar> v){  x = v[0]; y =v[1]; z=v[2];}
        Vector2 xy()const{return Vector2(x,y); }
        Vector2 xz()const{return Vector2(x,z); }
        Vector2 yz()const{return Vector2(y,z); }
        scalar max()const{ return std::max(x,std::max(y,z));}
        scalar min()const{ return std::min(x,std::min(y,z));}
        
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
        
        scalar angle(const Vector3& with = Vector3::Head)const { return acos(dot(with)/length()/with.length());}
        
        Vector3 interpolated(Vector3& to,scalar by){
            return (*this) + (to - (*this)) * by;
        }
        
        scalar pitch()const;
        
        scalar roll()const;
        
        scalar yaw() const;
        
        Quaternion rotation(const Vector3& to) const;
        
        Quaternion rotation(const Vector3& to,const Vector3& axis) ;
        Vector3 rotated(scalar angle,const Vector3& axis) ;
        
        Vector3 orthogonal()const
        {
            scalar xx = std::abs(x);
            scalar yy = std::abs(y);
            scalar zz = std::abs(z);
            
            Vector3 other = xx < yy ? (xx < zz ? Vector3::X : Vector3::Z) : (yy < zz ? Vector3::Y : Vector3::Z);
            return (*this).cross(other);
        }
        
        //Vector3 operator=(const Vector2& v2){   return (*this) =  Vector3(v2.x,v2.y,0); }
        
        Vector3 operator -()const{ return Vector3(-x,-y,-z);}
        friend Vector3 operator +(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x+v2.x,v1.y+v2.y,v1.z+v2.z); }
        friend Vector3 operator -(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x-v2.x,v1.y-v2.y,v1.z-v2.z); }
        friend Vector3 operator -(const scalar p,const Vector3& v2){ return  Vector3(p-v2.x,p-v2.y,p-v2.z); }
        friend void operator +=(Vector3& v1,const Vector3& v2){ v1 =  Vector3(v1.x+v2.x,v1.y+v2.y,v1.z+v2.z); }
        friend void operator -=(Vector3& v1,const Vector3& v2){  v1 =  Vector3(v1.x-v2.x,v1.y-v2.y,v1.z-v2.z); }
        
        
        
        friend Vector3 operator /(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x/v2.x,v1.y/v2.y, v1.z/v2.z); }
        friend Vector3 operator *(const Vector3& v1,const Vector3& v2){ return  Vector3(v1.x*v2.x , v1.y*v2.y , v1.z*v2.z); }
        friend Vector3 operator /(const Vector3& v1,const scalar v){ return  Vector3(v1.x/v,v1.y/v, v1.z/v); }
        friend Vector3 operator *(const Vector3& v1,const scalar v){ return  Vector3(v1.x*v , v1.y*v , v1.z*v); }
        friend Vector3 operator *(const scalar v,const Vector3& v1){ return  Vector3(v1.x*v , v1.y*v , v1.z*v); }
        friend bool operator ==(const Vector3& v1,const Vector3& v2){ return  std::abs(v1.x- v2.x) < eps && std::abs(v1.y - v2.y) < eps && std::abs(v1.z - v2.z) < eps; }
        friend bool operator !=(const Vector3& v1,const Vector3& v2){ return  !(v1 == v2); }
        
        friend bool operator <(const Vector3& v1,const scalar& v){ return  std::abs(v1.x) < v && std::abs(v1.y) < v && std::abs(v1.z) < v; }
        
        friend ostream& operator <<( ostream& out,const Vector3& v){ return out<<v.x<<"\t"<<v.y<<"\t"<<v.z; }
        friend istream& operator >>( istream& in,Vector3& v){
            if(in && in>>v.x){
                in>>v.y>>v.z;
            }
            return in;
        }
        friend ostream& operator <<( ostream& out,const vector<Vector3>& list){
            for(auto i : list){ out<<i<<"\n";   }
            out.flush();
            return out;
        }
        friend istream& operator >>( istream& in,vector<Vector3>& list){
            Vector3 v;
            if(in && in>>v.x){
                in>>v.y>>v.z;
                list.push_back(v);
            }
            return in;
        }
        
        
        
        
        operator vector<scalar>()const{ return array();}
        Vector3& operator =(const vector<scalar>& v){       x = v[0]; y = v[1]; z = v[2];   return *this;    }
        Vector3& operator =(scalar v){  x = v; y = v; z = v;    return *this;    }
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
        inline scalar _roll() const {
            //roll in regular
            auto t0 = 2 * (w * x + y * z);
            auto t1 = 1 - 2 * (x * x + y*y);
            return atan2(t0, t1);
            
            //        auto t0 = 2 * (w * x + y * z);
            //        auto t1 = w*w + z*z - x*x * y*y;// - 2 * (x * x + y*y);
            //        return atan2(t0, t1);
            //return atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
        }
        
        inline scalar _yaw()const{
            auto t3 = 2 * (w * z + x * y);
            auto t4 = 1 - 2 * (y*y + z * z);
            return atan2(t3, t4);
            // return asin(-2 * (x * z - w * y))
        }
        
        inline scalar _pitch() const{
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
        
        Quaternion(){ }
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
        Quaternion(Vector3 gyro,RotationOrder order = RotationOrder::ypr):Quaternion(gyro.y,gyro.x,gyro.z,order) {}
#else
        Quaternion(Vector3 gyro,RotationOrder order = RotationOrder::ypr):Quaternion(gyro.x,gyro.y,gyro.z,order) {}
#endif
        Quaternion(scalar roll,scalar pitch,scalar yaw,RotationOrder order = RotationOrder::ypr) { // default ypr
#ifdef MOBILE
            auto quatPitch = Quaternion(Vector3::X, pitch);
            auto quatYaw = Quaternion(Vector3::Z, yaw);
            auto quatRoll = Quaternion(Vector3::Y, roll);
#else
            auto quatPitch = Quaternion(Vector3::Y, pitch);
            auto quatYaw = Quaternion(Vector3::Z, yaw);
            auto quatRoll = Quaternion(Vector3::X, roll);
#endif
            
            //        vector<Vector3> axices = {Vector3::Z,Vector3::Y,Vector3::X};
            //        vector<scalar> angles = {yaw,pitch,roll};
            //        *this = Quaternion::Identity;
            //        auto& q = *this;
            //        for(int i = 0;i<axices.size();i++) {
            //            q = q*Quaternion(axices[i]*q,angles[i]);
            //        }
            //        return;
            // (*this) = q;
            
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
        void xyz(const Vector3& v) {  x = v.x;y = v.y; z=v.z;}
        
        Quaternion normalized() {
            auto ls = lengthSquared();
            if(std::abs(ls) < eps || std::abs(ls - 1) < eps ) {
                return (*this);
            }
            return (*this) / sqrt(ls);
        }
        
        Vector3 euler()const {
#ifdef MOBILE
            return Vector3((*this).pitch(),(*this).roll(),(*this).yaw());
#else
            return Vector3((*this).roll(),(*this).pitch(),(*this).yaw());
#endif
        }
        //    Quaternion normalizedWithAxis() {
        //        auto ls = lengthSquared();
        //        auto rtn = *this;
        //        if(std::abs(ls) < eps || std::abs(ls - 1) < eps ) {
        //            return (*this);
        //        }
        //        if( std::abs(w - 1) < eps ) {
        //            rtn.xyz(Vector3::Zero);
        //            return rtn;
        //        }
        //        return (*this) / sqrt(1-w*w);
        //    }
        void normalize() {
            //        auto ls = lengthSquared();
            //        if(std::abs(ls) < eps || std::abs(ls - 1) < eps ) {   return; }
            //        (*this) = (*this) / sqrt(ls);
            (*this) = normalized();
        }
        Vector3 axis()const {return Vector3(x,y,z).normalized();}
        scalar angle()const {return acos(w) * 2;}
        scalar roll() const{
#ifdef MOBILE
            return _pitch();
#else
            return _roll();
#endif
        }
        
        scalar yaw()const{
            //#ifdef MOBILE
            //        return -_yaw();
            //#else
            return _yaw();
            //#endif
        }
        
        scalar pitch() const{
#ifdef MOBILE
            return _roll();
#else
            return  _pitch();
#endif
        }
        
        scalar dot(const Quaternion& v)const{
            return x * v.x + y * v.y + z * v.z + w * v.w;
        }
        Quaternion withYaw(scalar a) const{
            auto e = euler();
            e.z = a;
            return Quaternion(e);
        }
        Quaternion withRoll(scalar a) const{
            auto e = euler();
#ifdef MOBILE
            e.y = a;
#else
            e.x = a;
#endif
            return Quaternion(e);
        }
        Quaternion withPitch(scalar a) const{
            auto e = euler();
#ifdef MOBILE
            e.x = a;
#else
            e.y = a;
#endif
            return Quaternion(e);
        }
        
        //    Quaternion lerp(const Quaternion& to, scalar p){
        //        scalar op = 1 - p;
        //        return Quaternion(op*x + p*to.x, op*y + p*to.y, op*z + p*to.z, op*w + p*to.w).normalized();
        //    }
        //    Quaternion slerp(const Quaternion& to,scalar p){
        //        scalar theta = acos(x*to.x + y*to.y + z*to.z + w*to.w);
        //        scalar sn = sin(theta);
        //        scalar a = sin((1-p)*theta) / sn;
        //        scalar b = sin(p*theta) / sn;
        //        return Quaternion(a*x + b*to.x,a*y + b*to.y,a*z + b*to.z,a*w + b*to.w).normalized();
        //    }
        
        Quaternion slerp(scalar p) const{
            auto v  = axis().normalized();
            auto a = angle();
            return Quaternion(v,p*a);
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
        friend Quaternion operator *(const Quaternion& q,const scalar v){
            return  q.slerp(v); //Quaternion(v1.x*v,v1.y*v,v1.z*v,v1.w*v); }
        }
        friend Quaternion operator *(const scalar v,const Quaternion& q){
            return  q*v;
        }
        friend void operator *=(Quaternion& q,const scalar v){ q = q*v; }
        friend bool operator ==(const Quaternion& v1,const Quaternion& v2){
            return  std::abs(v1.x- v2.x) < eps && std::abs(v1.y - v2.y) < eps && std::abs(v1.z - v2.z) < eps && std::abs(v1.w - v2.w) < eps;
        }
        friend bool operator !=(const Quaternion& v1,const Quaternion& v2){
            return  std::abs(v1.x- v2.x) > eps || std::abs(v1.y - v2.y) > eps || std::abs(v1.z - v2.z) > eps || std::abs(v1.w - v2.w) > eps;
        }
    };
    
    
    std::vector<Vector3> operator+(const std::vector<Vector3> &a, const std::vector<Vector3> &b);
    std::vector<Vector3> &operator+=(std::vector<Vector3> &a, const std::vector<Vector3> &b);
    
    
    template <typename T>
    std::vector<T> operator+(const std::vector<T> &A, const std::vector<T> &B)
    {
        std::vector<T> AB;
        AB.reserve( A.size() + B.size() );                // preallocate memory
        AB.insert( AB.end(), A.begin(), A.end() );        // add A;
        AB.insert( AB.end(), B.begin(), B.end() );        // add B;
        return AB;
    }
    
    template <typename T>
    std::vector<T> &operator+=(std::vector<T> &A, const std::vector<T> &B)
    {
        A.reserve( A.size() + B.size() );                // preallocate memory without erase original data
        A.insert( A.end(), B.begin(), B.end() );         // add B;
        return A;                                        // here A could be named AB
    }
    
    template <typename T>
    std::vector<T> operator+(const std::vector<T> &A,  std::initializer_list<T> B)
    {
        std::vector<T> AB;
        AB.reserve( A.size() + B.size() );                // preallocate memory
        AB.insert( AB.end(), A.begin(), A.end() );        // add A;
        AB.insert( AB.end(), B.begin(), B.end() );        // add B;
        return AB;
    }
    template <typename T>
    std::vector<T> operator+( std::initializer_list<T> A, const std::vector<T> &B)
    {
        return B+A;
    }
    
    vector<scalar> ternary(vector<scalar> start, vector<scalar> end, function<scalar(vector<scalar>)> eval);
    
    
}
