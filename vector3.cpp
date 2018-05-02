//
//  vector3.cpp
//  vector3
//
//  Created by Maksim Piriyev on 4/28/18.
//  Copyright © 2018 Dronee. All rights reserved.
//

#include "vector3.h"

namespace vector3{
        
    const Vector2 Vector2::Zero(0,0);
    const Vector2 Vector2::X(1,0);
    const Vector2 Vector2::Y(0,1);

    const Vector3 Vector3::Zero(0,0,0);
    const Vector3 Vector3::X(1,0,0);
    const Vector3 Vector3::Y(0,1,0);
    const Vector3 Vector3::Z(0,0,1);
    #ifdef MOBILE
    const Vector3 Vector3::Head(0,1,0);
    #else
    const Vector3 Vector3::Head(1,0,0);
    #endif

    const Quaternion Quaternion::Zero(0,0,0,0);
    const Quaternion Quaternion::Identity(0,0,0,1);



    scalar Vector3::pitch() {
        return Vector3::Head.rotation(*this).pitch();
    }

    scalar Vector3::roll() {
        return Vector3::Head.rotation(*this).roll();
    }

    scalar Vector3::yaw() {
        return Vector3::Head.rotation(*this).yaw();
    }

    Quaternion Vector3::rotation(const Vector3& to)const {
        auto a = cross(to);
        auto w = sqrt(lengthSquared() * to.lengthSquared()) + dot(to);
        auto q = Quaternion(a.x,a.y,a.z,w);
    //    auto ll = to.length();
    //    auto l = q.length();
        return q.normalized();
    }
    //Quaternion Vector3::rotation(const Vector3& to)const {
    //    auto a = cross(to).normalized();
    //    auto w =normalized().dot(to.normalized());
    //    auto q = Quaternion(a,acos(w));
    //    //    auto ll = to.length();
    //    //    auto l = q.length();
    //    return q.normalized();
    //}

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

}
