
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

#ifndef _SHAPESHITFER_LISTENER_
#define _SHAPESHITFER_LISTENER_

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

class ShapeShifterListener: public Manager {
protected:
    ros::Subscriber sub;
    ros::Publisher pub;
    bool started;
public:

    ShapeShifterListener(ros::NodeHandle & m, unsigned int port){
        init(m, port);
    }
    ShapeShifterListener(){}

    virtual void init(ros::NodeHandle & m, unsigned int port){
        this->n = m;
        this->port = port;
        started = true;
        type = SS_LISTENER;
    }


    virtual std::string get_in_topic(){
        std::ostringstream s;
        s << n.getNamespace() << "/tx/" << topic;
        return s.str();
    }

    virtual std::string get_out_topic(std::string topic, int sender){
        std::ostringstream s;
        s << n.getNamespace() << "/rx/R" << (int) sender << "/" << topic;
        return s.str();
    }

    void start(){
        started = true;
    }


    void stop(){
        started = false;
    }

    virtual void run() {
        boost::thread(boost::bind(&ShapeShifterListener::listen, this));
    }

    virtual bool pop(boost::shared_ptr<topic_tools::ShapeShifter> & shapeShifter, std::string & name, int & sender){
        unsigned int size;
        unsigned char src, priority;
        char * data;
        void * e = bz_pop(port,0, data, size, src,priority);

        sender = src;

        WrapperHeader * wh = (WrapperHeader *) data;
        ShapeShifterMessage * ssm = (ShapeShifterMessage *) &wh->data[0];
        ros::serialization::IStream istream((uint8_t *)&ssm->data[0], ssm->size);

        std::string metadata(&ssm->data[0] + ssm->size);
        shapeShifter->morph(metadata.substr(0,32), metadata.substr(32, ssm->datatype_size), metadata.substr(32 + ssm->datatype_size, ssm->msg_def_size), "false");
        shapeShifter->read(istream);

        name = metadata.substr(32 + ssm->datatype_size + ssm->msg_def_size, ssm->topic_name_size);

        if (e == NULL){
            ROS_WARN_ONCE("Fuction pop must NOT return null for the data to be used");
            return false;
        }

        bz_pop_done(e);
        return true;
    }

    struct publisher_t{
        ros::Publisher pub;
        std::string md5, message_def, type;
    };

    std::map<std::string, publisher_t > publishers;

    virtual void listen() {
        boost::shared_ptr<topic_tools::ShapeShifter> shapeShifter(new topic_tools::ShapeShifter);
        std::string name;
        int sender;
        while (ros::ok()) {
            if (pop(shapeShifter, name, sender)){
                std::cerr << "popped on PORT " << port <<  get_out_topic(name, sender) << std::endl;

                if (publishers.find(name) == publishers.end()){
                    publishers[name].pub = shapeShifter->advertise(n,get_out_topic(name, sender),1);
                    publishers[name].md5 = shapeShifter->getMD5Sum();
                    publishers[name].message_def = shapeShifter->getMessageDefinition();
                    publishers[name].type = shapeShifter->getDataType();
                }
                publishers[name].pub.publish(shapeShifter);
            }
        }
    }
};
#endif
