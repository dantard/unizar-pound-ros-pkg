
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

#ifndef _SHAPESHITFER_SENDER_
#define _SHAPESHITFER_SENDER_

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
#include <boost/regex.hpp>

class ShapeShifterSender: public Manager {
protected:
    ros::Subscriber sub;
    ros::Publisher pub;
    unsigned int dest;
    bool started;
    std::string dest_str;
public:

    ShapeShifterSender(ros::NodeHandle & m, unsigned int port, std::string topic, std::string dest, unsigned char priority, unsigned short deadline, unsigned short period){
        init(m, port, topic,dest,priority,deadline, period);
    }
    ShapeShifterSender(){}

    virtual void init(ros::NodeHandle & m, unsigned int port, std::string topic, std::string dest, unsigned char priority, unsigned short deadline, unsigned short period){
        this->n = m;
        this->port = port;
        this->priority = priority;
        this->deadline = deadline;
        this->period = period;
        this->topic = topic;
        this->type = TOPIC;
        this->dest_str = dest;
        parse_ids(dest_str, dests);
        this->dest = compute_broadcast_destination(dests);
        started = true;
    }

    virtual std::string get_in_topic(){
        std::ostringstream s;
        s << n.getNamespace() << "/tx/" << topic;
        return s.str();
    }

    void start(){
        started = true;
    }

    void stop(){
        started = false;
    }

    virtual void run() {
        wrapper_flow_add(port, period, priority, deadline);
        sub = n.subscribe(get_in_topic(), 1000, &ShapeShifterSender::callback, this);        
    }

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
            std::ostringstream oss, aux;

            /* Remove comments from message definitions */
            std::string md = boost::regex_replace(msg->getMessageDefinition(), boost::regex("[' ']{2,}"), " ");
            std::istringstream f(md);
            std::string line;
            while (std::getline(f, line)) {
               int pos = line.find("#");
                if (pos == std::string::npos){
                    aux << line << std::endl;
                }else if (pos != 0){
                    aux << line.substr(0,pos - 1)  << std::endl;
                }
            }
            /* done */

            oss << msg->getMD5Sum() << msg->getDataType() << aux.str() /*msg->getMessageDefinition()*/ << topic;
            std::vector<uint8_t> buffer(sizeof(WrapperHeader) + sizeof(ShapeShifterMessage) + msg->size() + oss.str().size());

            WrapperHeader * wh = (WrapperHeader *) &buffer[0];
            ShapeShifterMessage * ssm = (ShapeShifterMessage *) &wh->data[0];

            ssm->size = msg->size();
            ssm->datatype_size = msg->getDataType().size();
            ssm->msg_def_size = aux.str().size(); //msg->getMessageDefinition().size();
            ssm->topic_name_size = topic.size();

            ros::serialization::OStream ostream((uint8_t *) &ssm->data[0], msg->size());
            msg->write(ostream);
            memcpy(&ssm->data[0] + msg->size(), oss.str().c_str(), oss.str().size());

            bz_push((char *) &buffer[0], buffer.size(), priority, port, wrapper_get_node_id(), dest, deadline);
        }
    }
};
#endif
