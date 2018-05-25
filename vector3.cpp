//
//  vector3.cpp
//  vector3
//
//  Created by Maksim Piriyev on 4/28/18.
//  Copyright © 2018 Dronee. All rights reserved.
//

#include "vector3.h"
#include <float.h>

namespace vector3{
    
    
    const Vector2 Vector2::Zero(0,0);
    const Vector2 Vector2::X(1,0);
    const Vector2 Vector2::Y(0,1);
    const Vector2 Vector2::One(1,1);
    
    
    
    const Vector3 Vector3::Zero(0,0,0);
    const Vector3 Vector3::X(1,0,0);
    const Vector3 Vector3::Y(0,1,0);
    const Vector3 Vector3::Z(0,0,1);
    
    const Vector3 Vector3::One(1,1,1);// = Vector3(1,1,1).normalized();
    const Vector3 Vector3::Max(DBL_MAX,DBL_MAX,DBL_MAX);
    const Vector3 Vector3::Min(DBL_MIN,DBL_MIN,DBL_MIN);
    
#ifdef MOBILE
    const Vector3 Vector3::Head(0,1,0);
    const Vector2 Vector2::Head(0,1);
#else
    const Vector3 Vector3::Head(1,0,0);
    const Vector2 Vector2::Head(1,0);
#endif
    
    const Quaternion Quaternion::Zero(0,0,0,0);
    const Quaternion Quaternion::Identity(0,0,0,1);
    
    Vector2::operator Vector3()const { return Vector3(x,y,0); }
    
    
    scalar Vector3::pitch() const{
        //        return atan2(z,yz().length());
        // return atan2 (y,xz().length()) ;
        //Vector3 surface = xz();
        //return surface.angle(*this);
        //return xz().angle();
        return Vector3::Head.rotation(*this).pitch();
    }
    
    scalar Vector3::roll()const {
        // return atan (x/yz().length());
        return Vector3::Head.rotation(*this).roll();
    }
    
    scalar Vector3::yaw() const{
        //#ifdef MOBILE
        //        return xy().angle();
        //#else
        //        return -xy().angle();
        //#endif
        //   return atan (y/xz().length()) ;
        //        return atan (z/yz().length());
        //return -xy().angle();
        return Vector3::Head.rotation(*this).yaw();
    }
    
    //    Quaternion Vector3::rotation(const Vector3& to)const {
    //        auto a = cross(to);
    //        auto w = sqrt(lengthSquared() * to.lengthSquared()) + dot(to);
    //        auto q = Quaternion(a.x,a.y,a.z,w);
    //    //    auto ll = to.length();
    //    //    auto l = q.length();
    //        return q.normalized();
    //    }
    Quaternion Vector3::rotation(const Vector3& to)const {
        auto t = to.normalized();
        auto f = (*this).normalized();
        auto a = cross(t);
        auto w =f.dot(t);
        if(t == -f){
            return Quaternion(orthogonal(),rad(180));
        }
        //auto q = Quaternion(a,acos(w));
        //    auto ll = to.length();
        //    auto l = q.length();
        return Quaternion(a,acos(w));//q;//.normalized();
    }
    
    //quat quat::fromtwovectors(vec3 u, vec3 v)
    //{
    //    float cos_theta = dot(normalize(u), normalize(v));
    //    float angle = acos(cos_theta);
    //    vec3 w = normalize(cross(u, v));
    //    return quat::fromaxisangle(angle, w);
    //}
    //Quaternion Vector3::rotation(Vector3& to)const {
    //    auto a = cross(to.normalized()).normalized();
    //    auto w = dot(to);
    //    auto angle = acos(w)/2;
    //    w = cos(angle);
    //    a = a *  sin(angle);
    //    auto q = Quaternion(a.x,a.y,a.z,w);
    //    auto ll = to.length();
    //    auto l = q.length();
    //    return q.normalized();
    //}
    
    Quaternion Vector3::rotation(const Vector3& to,const Vector3& axis) {
        auto uv1 = axis.cross(*this);
        auto uv2 = axis.cross(to);
        auto w = (uv1+uv2).normalized().dot(uv1);
        return  Quaternion(axis.x,axis.y,axis.z,w);
    }
    Vector3 Vector3::rotated(scalar angle,const Vector3& axis) {
        auto q = Quaternion(axis, angle);
        return  (*this) * q;
    }
    
    vector<scalar> mid(vector<scalar>& a,vector<scalar>& b,int i,scalar m){
        vector<scalar> r(a.size());
        for(int j=0;j<a.size();j++) {
            r[j] = (a[j]+b[j])/2;
        }
        r[i] = m;
        return r;
    }
    vector<scalar> ternary(vector<scalar> start, vector<scalar> end, function<scalar(vector<scalar>)> eval){
        auto l = start, r = end;
        auto sum = [&](){
            auto rtn = 0.0;
            for(int i = 0;i<l.size();i++) rtn += r[i]-l[i];
            return rtn;
            
        };
        
        while(sum() > l.size()*eps){
            for(int j=0;j<start.size();j++) {
                auto l1 = (l[j]*2+r[j])/3;
                auto l2 = (l[j]+2*r[j])/3;
                if (eval(mid(l,r,j,l1)) < eval(mid(l,r,j,l2)) ){
                    r[j] = l2;
                }else{
                    l[j] = l1;
                }
                
            }
        }
        return l;
        
    }
    
    //    vector<scalar> ternary(vector<scalar> start, vector<scalar> end, function<scalar(vector<scalar>)> eval){
    //        auto l = start, r = end;
    //        auto prevEval = eval(l);
    //        for(int i=0;i<100*start.size();i++){
    //            for(int j=0;j<start.size();j++) {
    //                auto l1 = (l[j]*2+r[j])/3;
    //                auto l2 = (l[j]+2*r[j])/3;
    //                if (eval(mid(l,r,j,l1)) < eval(mid(l,r,j,l2)) ){
    //                    r[j] = l2;
    //                }else{
    //                    l[j] = l1;
    //                }
    //
    //            }
    //        }
    //        return l;
    //
    //    }
    
    
    std::vector<Vector3> operator+(const std::vector<Vector3> &A, const std::vector<Vector3> &B)
    {
        std::vector<Vector3> AB;
        AB.reserve( A.size() + B.size() );                // preallocate memory
        AB.insert( AB.end(), A.begin(), A.end() );        // add A;
        AB.insert( AB.end(), B.begin(), B.end() );        // add B;
        return AB;
    }
    
    std::vector<Vector3> &operator+=(std::vector<Vector3> &A, const std::vector<Vector3> &B)
    {
        A.reserve( A.size() + B.size() );                // preallocate memory without erase original data
        A.insert( A.end(), B.begin(), B.end() );         // add B;
        return A;                                        // here A could be named AB
    }
}
