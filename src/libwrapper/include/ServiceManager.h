
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

#ifndef _SERVICE_MANAGER_
#define _SERVICE_MANAGER_

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <map>


#define SET_BIT(array, bit)   (array |= (1 << bit))

#include <ros/ros.h>
#include "../interface/interface.h"
#include "Manager.h"

template<class T> class ServiceManager : public Manager {
    ros::ServiceServer service;
    bool im_server;
    unsigned short serial;

    struct service_data_t{
      bool response;
      bool response2;
      unsigned short serial;
      char data[0];
    };

public:
    ServiceManager(ros::NodeHandle & m, unsigned int port, std::string topic, unsigned char server, unsigned char priority, unsigned short deadline){
        this->n = m;
        this->port = port;
        this->priority = priority;
        this->deadline = deadline;
        this->topic = topic;
        this->server = server;
        type = SERVICE;
    }

    virtual void run() {

        im_server = server == wrapper_get_node_id();
        wrapper_flow_add(port, 0, priority, deadline);
        wrapper_flow_add(port+1, 0, priority, deadline);

        if (im_server) {
            boost::thread(boost::bind(&ServiceManager::listen, this));
        }else{
            std::ostringstream srv_topic;
            srv_topic << n.getNamespace() <<"/REM/R" << (int) server << "/" << topic;
            service = n.advertiseService(srv_topic.str(), &ServiceManager::callback, this);
        }
    }


    bool callback(typename T::Request &req, typename T::Response &resp) {

        ros::SerializedMessage buff = ros::serialization::serializeMessage<typename T::Request>(req);
        char message[sizeof(service_data_t) + buff.num_bytes];

        /* Step 1: I receive a service call and send the request to the server node */
        service_data_t * sd = (service_data_t *) message;
        memcpy(sd->data, buff.message_start, buff.num_bytes);
        sd->serial = serial ++;

        wrapper_push(message, sizeof(service_data_t) + buff.num_bytes, priority, port, wrapper_get_node_id(), 1 << server, deadline);

        /* Step 3: I wait for the answer of the server node */
        bool successful = false;
        unsigned short recvd_serial = 0;
        do {
            char * data; unsigned int size; unsigned char src; unsigned char prio;
            void * e = wrapper_pop(port + 1, deadline*3, data, size, src, prio);
            if (e == NULL){
                std::cerr << "timeout" << std::endl;
                return false;
            }

            sd = (service_data_t *) data;
            size = size - sizeof(service_data_t);

            typename T::Response response;
            if (!deserialize<typename T::Response>(sd->data, size, response)) {
                wrapper_pop_done(e);
                return false;
            }

            resp = response;
            successful = sd->response;
            recvd_serial = sd->serial;

            wrapper_pop_done(e);

        } while (serial != (recvd_serial + 1));

        return successful;
    }

    virtual void listen() {

        typename T::Request request;
        ros::ServiceClient client = n.serviceClient<T> (topic);
        while (ros::ok()) {
            unsigned int size;
            unsigned char src, priority;
            char * data;

            /* Step 2: I receive a service execution request*/

            void * e = wrapper_pop(port, 0, data, size, src, priority);
            if (e == NULL){
                continue;
            }

            service_data_t * sd = (service_data_t *) data;
            size = size - sizeof(service_data_t);
            int serial = sd->serial;

            if (!deserialize<typename T::Request>(sd->data, size, request)) {
                wrapper_pop_done(e);
                return;
            }
            wrapper_pop_done(e);

            /* Step 3: I execute the call and send the result back */
            T call;
            call.request = request;
            bool response = client.call(call);


            ros::SerializedMessage buff = ros::serialization::serializeMessage<typename T::Response>(call.response);

            char message[sizeof(service_data_t) + buff.num_bytes];
            sd = (service_data_t *) message;

            sd->serial = serial;
            sd->response = response;
            memcpy(sd->data, buff.message_start, buff.num_bytes);

            unsigned int dest = 0;
            SET_BIT(dest, src);

            wrapper_push(message, sizeof(service_data_t) + buff.num_bytes, priority, port + 1, wrapper_get_node_id(), dest , deadline);

        }
    }

};
#endif
