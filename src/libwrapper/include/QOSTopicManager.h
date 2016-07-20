
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
#ifndef _QOS_TOPIC_MANAGER_
#define _QOS_TOPIC_MANAGER_
#include "TopicManager.h"

template<class T> class QOSTopicManager: public TopicManager<T> {
int queue_length;

public:
    QOSTopicManager(ros::NodeHandle & m, unsigned int port, std::string topic, unsigned char src, std::string dest, unsigned char priority, unsigned short deadline, unsigned short period, int queue_length){
        this->init(m,port, topic,src,dest,priority,deadline, period);
        this->queue_length = queue_length;
    }

    virtual void listen() {
        T pm;
        bool started = false;
        while (ros::ok()) {
            int nelem = wrapper_get_num_of_enqueued_elements(this->port);
            if (nelem > queue_length){
                started = true;
            }

            if (started && nelem > 0){
                if (this->pop(pm)){
                    this->pub.publish(pm);
                }
            }else{
                started = false;
            }
            usleep(this->period * 1000);
        }
    }
};
#endif
