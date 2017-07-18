
/*  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2016, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed in the hope that it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#ifndef _MANAGER_H__
#define _MANAGER_H__

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <map>

#define SET_BIT(array, bit)   (array |= (1 << bit))

#include <ros/ros.h>
#include "../interface/interface.h"
#include "../include/TimeSpec.h"
#include "ShapeShifterMessage.h"
#include "bzlib.h"

class Manager {
public:
    enum obj_type_t {TOPIC, SERVICE, SS_LISTENER};

protected:
    unsigned int port;
    unsigned char priority;
    unsigned short deadline;
    unsigned short period;
    ros::NodeHandle n;
    std::string topic;
    unsigned char server;
    obj_type_t type;
    int max_message_length = 0;
    int byte_total = 0;
    bool compress;
    std::vector<int> sources, dests;

public:
    int get_max_message_length(){
        return max_message_length;
    }

    int get_total_bytes(){
        return byte_total;
    }

    virtual void reconnect(){}
    virtual void start(){}
    virtual void stop(){}
    virtual void run(){}


    unsigned char get_server(){
        return server;
    }

    obj_type_t get_type(){
        return type;
    }

    std::string get_topic(){
        return topic;
    }
    virtual int get_port(){
        return port;
    }
    virtual int get_period(){
        return period;
    }

    virtual int set_period(int period){
        return this->period = wrapper_set_period(port, period);
    }

    virtual int get_priority(){
        return priority;
    }

    virtual int set_priority(int priority){
        return this->priority = wrapper_set_priority(port, priority);
    }


    std::vector<int> & get_destinations(){
        return dests;
    }

    bool parse_ids(std::string source, std::vector<int> & ids){
        bool im_in = false;
        std::vector<std::string> strs;
        boost::split(strs, source, boost::is_any_of(" ,n/"));
        for (unsigned i = 0; i < strs.size();i++){
            int id = atoi(strs.at(i).c_str());
            ids.push_back(id);
            if (id == wrapper_get_node_id()){
                im_in = true;
            }
        }
        return im_in;
    }

    Manager(){
        compress = 0;
    }
    unsigned int compute_broadcast_destination(std::vector<int> & ids){
        unsigned int dest = 0;
        for (unsigned i = 0; i < ids.size();i++){
            SET_BIT(dest, ids[i]);
        }
        return dest;
    }

    template <typename Q> bool deserialize(char * p, int size, Q & pm){

        static boost::shared_array<uint8_t> dbuff;
        static int current_size = 0;
        if (size > current_size){
            dbuff.reset(new uint8_t[size]);
            current_size = size;
        }
        memcpy(dbuff.get(),p, size);

        try {
            ros::SerializedMessage smsg(dbuff, size);
            ros::serialization::deserializeMessage<Q>(smsg, pm);
            ros::Time* tm = ros::message_traits::timeStamp(pm);
            if (tm){
                ROSWMP_DEBUG(stderr,"Type %s, tm", ros::message_traits::datatype(pm));
                *tm = ros::Time::now();
            }else{
                ROSWMP_DEBUG(stderr,"Type %s, no TS", ros::message_traits::datatype(pm));
            }

            return true;
        } catch (...){
            ROS_ERROR("DESERIALIZE ERROR IN %s\n", topic.c_str());
            return false;
        }
    }

    void set_compression(bool value){
        compress = value;
    }

    bool get_compression(){
        return compress;
    }

    int bz_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned int dst, unsigned int deadline){

        WrapperHeader * wh = (WrapperHeader *) data;
        wh->uncompressed_size = 0;

        if (compress){
            unsigned int payload = size - sizeof(WrapperHeader);
            unsigned int cmp_size = payload;

            char * buff = new char [cmp_size];
            int bz_result = BZ2_bzBuffToBuffCompress(buff, &cmp_size, &wh->data[0], payload, 1, 0, 30);

            if (bz_result == BZ_OUTBUFF_FULL){
                ROS_INFO_ONCE("Port %d -> negative compression, sending uncompressed", port);
            }else if (bz_result != 0){
                ROS_WARN_THROTTLE(1000,"Port %d -> compression error (%d), sending uncompressed", port, bz_result);
            }else{
                wh->uncompressed_size = payload;
                memcpy(&wh->data[0], buff, cmp_size);
                size = cmp_size + sizeof(WrapperHeader);
            }
            delete [] buff;
        }
        return wrapper_push(data, size, priority, port, src,dst,deadline);
    }

    void * bz_pop(int port, int timeout, char* & data, unsigned int & size, unsigned char & src, unsigned char & priority){


        std::pair<void *, char * > * pnt = new std::pair<void *, char *>();
        pnt->first = wrapper_pop(port, timeout, data, size, src, priority);
        pnt->second = 0;

        WrapperHeader * wh1  = (WrapperHeader *) data;

        if (wh1->uncompressed_size >0){
            WrapperHeader * wh2 = (WrapperHeader *) new char[wh1->uncompressed_size + sizeof(WrapperHeader)];
            int bz_result = BZ2_bzBuffToBuffDecompress(&wh2->data[0], &wh1->uncompressed_size, &wh1->data[0], size - sizeof(WrapperHeader), 0, 0);
            if (bz_result != 0){
                ROS_ERROR_THROTTLE(1000,"Port %d -> decompression error (%d)", port, bz_result);
                pnt->first = 0;
            }else{
                data = pnt->second = (char *) wh2;
                size = wh1->uncompressed_size;
                memcpy(wh2, wh1, sizeof(WrapperHeader));
            }
        }
        return pnt;
    }

    int bz_pop_done(void * data){
        std::pair<void *, char * > * pnt = (std::pair<void *, char *> *) data;
        if (pnt->second){
            delete pnt->second;
        }
        return wrapper_pop_done(pnt->first);
    }

};
#endif
