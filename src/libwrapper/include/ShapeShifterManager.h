
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

#ifndef _SHAPESHITFER_MANAGER_
#define _SHAPESHITFER_MANAGER_

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <map>

#include <ros/ros.h>
#include "../interface/interface.h"
#include "Manager.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <topic_tools/shape_shifter.h>
#include <streambuf>
#include "ShapeShifterMessage.h"

class ShapeShifterManager: public Manager {
protected:
    ros::Subscriber sub;
    ros::Publisher pub;
    std::vector<int> sources, dests;
    unsigned int dest;
    bool im_source, im_dest;
    bool started;
    unsigned char src;
    std::string dest_str;
public:

    ShapeShifterManager(ros::NodeHandle & m, unsigned int port, std::string topic, unsigned char src, std::string dest, unsigned char priority, unsigned short deadline, unsigned short period){
        init(m, port, topic,src,dest,priority,deadline, period);
    }
    ShapeShifterManager(){}

    virtual void init(ros::NodeHandle & m, unsigned int port, std::string topic, unsigned char src, std::string dest, unsigned char priority, unsigned short deadline, unsigned short period){
        this->n = m;
        this->port = port;
        this->priority = priority;
        this->deadline = deadline;
        this->period = period;
        this->topic = topic;
        this->server = src;     
        this->type = TOPIC;
        this->src = src;
        this->dest_str = dest;
        started = true;
    }


    virtual std::string get_in_topic(){
        std::ostringstream s;
        s << n.getNamespace() << "/tx/" << topic;
        return s.str();
    }

    virtual std::string get_out_topic(){
        std::ostringstream s;
        s << n.getNamespace() << "/rx/R" << (int) server << "/" << topic;
        return s.str();
    }

    void start(){
        started = true;
    }


    void stop(){
        started = false;
    }

    virtual void run() {
        im_source = src == wrapper_get_node_id();
        im_dest = parse_ids(dest_str, dests);
        this->dest = compute_broadcast_destination(dests);
        wrapper_flow_add(port, period, priority, deadline);

        if (im_source) {
            sub = n.subscribe(get_in_topic(), 1000, &ShapeShifterManager::callback, this);
        }
        if (im_dest){
//            pub = n.advertise<T> (get_out_topic(),1000);
            boost::thread(boost::bind(&ShapeShifterManager::listen, this));
        }
    }

//    virtual unsigned char getPriority(const boost::shared_ptr<T const> & message){
//        return priority;
//    }
//    virtual unsigned char getPriority(T & message){
//        return priority;
//    }
//    virtual unsigned int getDest(T & message){
//        return dest;
//    }
//    virtual unsigned int getDest(const boost::shared_ptr<T const> & message){
//        return dest;
//    }

    std::vector<unsigned int> ticks;
    int count = 0;

    bool is_time_to_push(){

        ticks.push_back(TimeSpec::timestamp_ms());
        if (ticks.size() > 5){
            ticks.erase(ticks.begin());
        }

        unsigned int sum = 0;
        for (int i = 0; i< ticks.size()-1; i++){
            sum += (ticks[i+1] - ticks[i]);
        }

        int period = ticks.size() > 1 ? sum/(ticks.size()-1): 0;
        if (period == 0 || this->period == 0){
            return true;
        }

        int rate = int(nearbyint(double(this->period)/double(period)));
        int real_period = rate*period;
        double error = fabs(1.0-(double(real_period)/double(this->period)));
        if (error > 0.05){
            ROS_WARN_ONCE("Topic '%s' real period is %dms", topic.c_str(), real_period);
        }
        if (++count > rate-1){
            count = 0;
            return true;
        }
        return false;
    }

    virtual void callback(const topic_tools::ShapeShifter::ConstPtr& message) {
        if (is_time_to_push()){
            push(message);
        }
    }

    virtual void push(const topic_tools::ShapeShifter::ConstPtr& msg){
        if (started){
            std::ostringstream oss;

            int extra_length =  + msg->getDataType().size() + msg->getMD5Sum().size() + msg->getMessageDefinition().size() + 2;
            std::vector<uint8_t> buffer(msg->size() + extra_length + sizeof(ShapeShifterMessage));
            ShapeShifterMessage * ssm = (ShapeShifterMessage *) &buffer[0];
            ssm->size = msg->size();
            ssm->datatype_size = msg->getDataType().size();
            ssm->msg_def_size = msg->getMessageDefinition().size();

            oss << msg->getMD5Sum() << msg->getDataType() << msg->getMessageDefinition();

            unsigned char * pointer = &buffer[0] + sizeof(ShapeShifterMessage);
            ros::serialization::OStream ostream(pointer, msg->size());
            msg->write(ostream);

            pointer += msg->size();
            memcpy(pointer, oss.str().c_str(), extra_length);
            fprintf(stderr,"test: %s\n",pointer);

            //https://github.com/juancamilog/rosserial-mrl/blob/master/rosserial_server/src/topic_handlers.h

            wrapper_push((char *) &buffer[0], buffer.size(), priority, port, wrapper_get_node_id(), dest, deadline);
        }
    }

//    virtual void push(T & message){
//        if (started){
//            ros::SerializedMessage buff = ros::serialization::serializeMessage<T>(message);
//            ROSWMP_DEBUG(stderr,"Serializing size %d\n",buff.num_bytes);
//            int priority = getPriority(message);
//            unsigned int dest = getDest(message);
//            wrapper_push((char *) buff.message_start, buff.num_bytes, priority, port, wrapper_get_node_id(), dest, deadline);
//        }
//    }

    virtual bool pop(boost::shared_ptr<topic_tools::ShapeShifter> & shapeShifter){
        unsigned int size;
        unsigned char src, priority;
        char * data;
        void * e = wrapper_pop(port,0, data, size, src,priority);

        ShapeShifterMessage * ssm = (ShapeShifterMessage *) data;
        unsigned char * pointer = (unsigned char *) data + sizeof(ShapeShifterMessage);
        ros::serialization::IStream istream(pointer, ssm->size);
        pointer += ssm->size;

        std::string metadata((const char * )pointer);

        shapeShifter->morph(metadata.substr(0,32), metadata.substr(32, ssm->datatype_size), metadata.substr(32 + ssm->datatype_size), "false");
        shapeShifter->read(istream);

        std::cerr << metadata.substr(0,32) << " --- " << metadata.substr(32, ssm->datatype_size) << "---" << metadata.substr(32 + ssm->datatype_size) << std::endl;

        if (e == NULL){
            ROS_WARN_ONCE("Fuction pop must NOT return null for the data to be used");
            return false;
        }

        wrapper_pop_done(e);
        return true;
    }
    virtual void listen() {
        boost::shared_ptr<topic_tools::ShapeShifter> shapeShifter(new topic_tools::ShapeShifter);
        while (ros::ok()) {
            if (pop(shapeShifter)){
                static ros::Publisher pub = shapeShifter->advertise(n,get_out_topic(),1);
                pub.publish(shapeShifter);
            }
        }
    }

};
#endif
