//
//  sensor.hpp
//  TestVector3
//
//  Created by Maksim Piriyev on 5/10/18.
//  Copyright © 2018 Maksim Piriyev. All rights reserved.
//

#pragma once

#include <vector>
#include <string>
#include <fstream>
#include "vector3.h"
using namespace vector3;
using namespace std;

enum SensorType{
    none = 0,accelerometer,gyroscope,magnetometer,coreMotion,position,velocity,userAcceleration,pixels
};
class SensorData{
public:
    double time = 0; // in secs
    SensorType type = SensorType::none;
    Vector3 v = Vector3::Zero;
    friend bool operator <(const SensorData& v1,const SensorData& v2){ return  v1.time == v2.time ? v1.type < v2.type : v1.time < v2.time; }
    
    friend ostream& operator <<( ostream& out,const SensorData& d){ return  out<<d.time<<"\t"<<d.v.x<<"\t"<<d.v.y<<"\t"<<d.v.z<<"\t"<<d.type; }
    friend istream& operator >>( istream& in,SensorData& d){
        if(in && in>>d.time){
                int type;
                in>>d.v>>type;
                d.type = (SensorType)type;
        }
        return in;
    }
    friend ostream& operator <<( ostream& out,const vector<SensorData>& list){
        for(auto i : list){ out<<i<<"\n";   }
        out.flush();
        return out;
    }
    friend istream& operator >>( istream& in,vector<SensorData>& list){
        SensorData d;
        if(in)
            while(in>>d.time){
                int type;
                in>>d.v>>type;
                d.type = (SensorType)type;
                list.push_back(d);
            }
        return in;
    }
    
    static void save(const vector<SensorData>& list,string path){
        ofstream out(path);
        out<<list;
        out.close();
    }
    static vector<SensorData> readAll(string path){
        vector<SensorData> rtn;
        ifstream in(path);
        in>>rtn;
        sort(rtn.begin(), rtn.end(),[](SensorData& a,SensorData& b){
            return a.time < b.time;
        });
        return rtn;
    }
    static vector<SensorData> filter(const vector<SensorData>& list,SensorType type){
        vector<SensorData> rtn;
        for(auto i : list){
            if(i.type == type){
                rtn.push_back(i);
            }
        }
        return rtn;
    }
};


