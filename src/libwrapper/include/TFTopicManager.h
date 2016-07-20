
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

#ifndef _TFTOPIC_MANAGER_
#define _TFTOPIC_MANAGER_

#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>
#include "TopicManager.h"

class TFTopicManager : public TopicManager<tf2_msgs::TFMessage> {

    std::vector<std::string> black_list;
    std::vector<std::string> white_list;
    std::string out_topic;
    tf2_msgs::TFMessage msg;
    boost::shared_ptr<tf2_msgs::TFMessage> msg_ptr;

    struct timespec ts;
public:
    TFTopicManager(ros::NodeHandle & m, unsigned int port, std::string topic, unsigned char src, std::string dest, unsigned char priority, unsigned short deadline, unsigned short period){
        init(m, port, topic,src,dest,priority,deadline, period);
        TimeSpec::now(&ts);
        out_topic = topic;
        msg_ptr.reset(&msg);
    }

    std::map<std::string, geometry_msgs::TransformStamped> map;

    virtual std::string get_in_topic(){
        return topic;
    }
    void set_out_topic(std::string name){
        out_topic = name;
    }
    virtual std::string get_out_topic(){
        return out_topic;
    }

    virtual void callback(const boost::shared_ptr<tf2_msgs::TFMessage const> & message) {
        for (int i = 0; i<message->transforms.size(); i++){
            geometry_msgs::TransformStamped ts = message->transforms.at(i);
            std::ostringstream oss;
            oss << ts.header.frame_id << "->" << ts.child_frame_id;

            if (black_list.size() > 0){
                if (std::find(black_list.begin(), black_list.end(), oss.str()) != black_list.end()){
                  continue;
                }
            }

            if (white_list.size() > 0){
                if (std::find(white_list.begin(), white_list.end(), oss.str()) == white_list.end()){
                  continue;
                }
            }

            map[oss.str()] = ts;
        }
        if (TimeSpec::elapsed_ms(&ts) > period){
            std::map<std::string, geometry_msgs::TransformStamped>::iterator it;
            msg.transforms.clear();
            for (it = map.begin(); it!=map.end(); it++){
                msg.transforms.push_back(it->second);
            }
            push(msg_ptr);
            TimeSpec::now(&ts);
        }
    }

    void add_blacklist(std::string elem){
        black_list.push_back(elem);
    }

    void add_whitelist(std::string elem){
        white_list.push_back(elem);
    }
};
#endif
