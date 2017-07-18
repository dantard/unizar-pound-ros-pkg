
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

#include <vector>

#include "include/ServiceManager.h"
#include "include/TFTopicManager.h"
#include "include/QOSTopicManager.h"
#include "include/TopicManager.h"
#include "include/ShapeShifterManager.h"
#include "include/ShapeShifterSender.h"
#include "include/ShapeShifterListener.h"

#include "ros/ros.h"
#include "libwrapper/Manage.h"
#include "include/argon.h"
#include "config/data_types.h"
#include <yaml-cpp/yaml.h>


#define VTOPIC(type, topic, source, dest, priority, period, time_to_live) map[topic] = new TopicManager<type>(n, port, topic,source,std::vector<int>(dest),priority,time_to_live, period); port++;
#define TOPIC(type, topic, source, dest, priority, period, time_to_live) map[topic] = new TopicManager<type>(n, port, topic,source,dest,priority,time_to_live, period); port++;
#define QOSTOPIC(type, topic, source, dest, priority, period, time_to_live, queue_length) map[topic] = new QOSTopicManager<type>(n, port, topic,source,dest,priority,time_to_live, period, queue_length); port++;
#define TFTOPIC(topic, source, dest, priority, period, time_to_live) map[topic] = new TFTopicManager(n, port, topic, source, dest, priority, time_to_live, period); port++;
#define SERVICE(type, topic, source, priority, time_to_live) map[topic] = new ServiceManager<type>(n, port, topic, source, priority, time_to_live); port+=2;
#define TOPIC_STOP(topic); map[topic]->stop();
#define SSTOPIC(type, topic, source, dest, priority, period, time_to_live) map[topic] = new ShapeShifterManager(n, port, topic,source,dest,priority,time_to_live, period); port++;
#define TF_ADD_WHITELIST(topic, elem); ((TFTopicManager *) map[topic])->add_whitelist(elem);
#define TF_ADD_BLACKLIST(topic, elem); ((TFTopicManager *) map[topic])->add_blacklist(elem);
#define TF_OUT_TOPIC(topic, elem); ((TFTopicManager *) map[topic])->set_out_topic(elem);

int node_id, num_of_nodes;
std::map<std::string, Manager * >::iterator it;
std::map<std::string, Manager * > map;
std::vector<Manager * > sms;

bool callback(typename libwrapper::Manage::Request &req, typename libwrapper::Manage::Response &resp) {

    std::transform(req.command.begin(), req.command.end(),req.command.begin(), ::toupper);

    if (req.command.at(0) == '_'){
        return wrapper_call_service(req.command, req.param1, req.param2, resp.info, resp.result);
    }else if(req.command.compare("LIST") == 0){
        resp.info.clear();
        resp.info.append("LIST, GET_NODE_ID, ECHO, START, STOP, RECONNECT, SET_PERIOD, SET_PRIORITY, GET_PRIORITY, GET_PERIOD");
        resp.result = 0;
        return true;
    }else if(req.command.compare("GET_NODE_ID") == 0){
        resp.info.append("OK");
        resp.result = wrapper_get_node_id();
        return true;
    }else if(req.command.compare("ECHO") == 0){
        resp.info.append(req.param1);
        resp.result = 0;
        return true;
    }

    auto it = map.find(req.param1);
    if (it == map.end()){
        resp.info.append("INEXISTENT TOPIC/SERVICE or COMMAND");
        resp.result = -1;
        return true;
    }

    if (it->second->get_server() != wrapper_get_node_id()){
        resp.info.append("TOPIC/SERVICE *NOT* ON THIS NODE");
        resp.result = -2;
        return true;
    }

    if (it->second->get_type() != Manager::TOPIC){
        resp.info.append("THIS COMMAND IS ONLY AVAILABLE FOR TOPICS");
        resp.result = -4;
        return true;
    }


    resp.info.append("OK");
    resp.result = 0;

    if (req.command.compare("STOP") == 0){

        it->second->stop();
        return true;

    }else if(req.command.compare("START") == 0){

        it->second->start();
        return true;

    }else if(req.command.compare("SET_PERIOD") == 0){

        resp.result = it->second->set_period(req.param2);

        return true;

    }else if(req.command.compare("SET_PRIORITY") == 0){

        resp.result = it->second->set_priority(req.param2);
        return true;

    }else if(req.command.compare("GET_PRIORITY") == 0){

        resp.result = it->second->get_priority();
        return true;

    }else if(req.command.compare("GET_PERIOD") == 0){

        resp.result = it->second->get_period();
        return true;
    }else if(req.command.compare("GET_MAX_MESSAGE_LENGTH") == 0){

        resp.result = it->second->get_max_message_length();
        return true;
    }else if(req.command.compare("GET_TOTAL_BYTES") == 0){

        resp.result = it->second->get_total_bytes();
        return true;
    }else if(req.command.compare("RECONNECT") == 0){
        it->second->reconnect();
        return true;
    }

    resp.info.append("INEXISTENT COMMAND");
    resp.result = -3;
    return true;

}

void create_shapeshifter_topics(ros::NodeHandle &n, int port, YAML::Node & topics){
    for (int i = 0; i<topics.size(); i++){
        YAML::Node topic = topics[i];
        std::string name;
        if (YAML::Node parameter  = topic["name"]){
            name = parameter.as<std::string>();
        }else{
            ROS_ERROR("Field 'name' is mandatory in a topic definition");
            exit(1);
        }
        std::string dest;
        if (YAML::Node parameter  = topic["dest"]){
            dest = parameter.as<std::string>();
        }else{
            ROS_ERROR("Field 'dest' is mandatory in a topic definition");
            exit(1);
        }
        int priority = 0;
        if (YAML::Node parameter  = topic["priority"]){
            priority = parameter.as<int>();
        }
        int period = 1000;
        if (YAML::Node parameter  = topic["period"]){
            period = parameter.as<int>();
        }
        int source = 0;
        if (YAML::Node parameter  = topic["source"]){
            source = parameter.as<int>();
        }
        int deadline = 1000;
        if (YAML::Node parameter  = topic["deadline"]){
            deadline = parameter.as<int>();
        }

        map[name] = new ShapeShifterSender(n, port, name, dest, priority, deadline, period);

        if (YAML::Node parameter  = topic["use_compression"]){
            bool use_compression = parameter.as<bool>();
            map[name]->set_compression(use_compression);
        }

    }
    std::cout << "Added " << topics.size() << " shapeshifter topics" << std::endl;

}

int main(int argc, char * argv[]){

    char ns[32], name[32], path[32];
    int ans, delay;
    argo_setCommentId(argo_addString(name, STR("node-name"), STR("")),
                      STR("Specify the name of the node"));
    argo_setCommentId(argo_addString(ns, STR("namespace"), STR("")),
                      STR("Specify the namespace"));
    argo_setCommentId(argo_addInt(&ans, STR("auto-namespace"), 0, 0),
                      STR("Obtain the namespace from node_id"));
    argo_setCommentId(argo_addString(path, STR("force-path"), STR("")),
                      STR("Specify the path of the messages"));
    argo_setCommentId(argo_addIntMandatory(&node_id, STR("node-id"), 0, 1),
                      STR("Specify node address"));
    argo_setCommentId(argo_addIntMandatory(&num_of_nodes, STR("num-of-nodes"), 2, 1),
                      STR("Specify number of nodes"));
    argo_setCommentId(argo_addInt(&delay, STR("start-delay"), 0, 1),
                      STR("Delays the start of the node"));
    argo_setExample(argv[0],STR("--node-id 0 --num-of-nodes 3 --auto-namespace"));

    argo_doProcess(argc, argv, 0);

    std::ostringstream oss;
    oss << "R" << node_id;
    ros::init(argc, argv, oss.str());

    ros::NodeHandle n(oss.str());

    int port = 0;
    for (int i = 0; i< num_of_nodes; i++) {
        Manager * sm = new ServiceManager<libwrapper::Manage>(n, port, "manage", i, 10, 21000);
        std::ostringstream oss;
        oss << "manage (R" << i << ")";
        map[oss.str()] = sm;
        sms.push_back(sm);
        port+=2;
    }
    ros::ServiceServer service = n.advertiseService("manage", callback);
    map["ss_listener"] = new ShapeShifterListener(n, port);

    std::ostringstream oss1;
    std::string filename = wrapper_get_config_filename();

    oss1 << getenv("HOME") << "/" <<filename;
    std::cout << "Reading config file " << oss1.str() << "..." << std::endl;
    try{

        YAML::Node config = YAML::LoadFile(oss1.str());
        wrapper_setup_config(config, node_id, num_of_nodes, port);

        if (YAML::Node topics = config["topics"]){
            create_shapeshifter_topics(n, port, topics);
        }

        std::cout << "Done." << std::endl;
    }catch(...){
        std::cout << "Bad file or file not found." << std::endl;
    }
    port++;

    #include "config/config.h"

    wrapper_init(n, node_id, num_of_nodes, port);

    printf("\n%-40s PORTS  PRIORITY  PERIOD   CMP   DEST\n", "TOPICS");
    printf("-----------------------------------------------------------------------------\n");
    for (it = map.begin(); it != map.end(); it++){
        if (it->second->get_type()==Manager::TOPIC){
            printf("%-40s %5d       %3d   %5d   %3s  ", it->first.c_str(), it->second->get_port(), it->second->get_priority(), it->second->get_period(),it->second->get_compression()?"yes":"no");

            for (int i = 0; i< it->second->get_destinations().size(); i++){
                printf("%s", i != 0 ? ", " : " ");
                printf("%d", it->second->get_destinations().at(i));
            }
            printf("\n");
        }


    }

    printf("\n%-40s PORTS  PRIORITY  PERIOD   CMP\n", "SERVICES");
    printf("----------------------------------------------\n");
    for (it = map.begin(); it != map.end(); it++){
        if (it->second->get_type()==Manager::SERVICE){
            printf("%-40s %2d,%2d       %3d   %5s   %3s\n", it->first.c_str(), it->second->get_port(), it->second->get_port()+1, it->second->get_priority(), "n/a",it->second->get_compression()?"yes":"no");
        }
    }
    printf("\n");
    wrapper_run_bg();


    for (it = map.begin(); it != map.end(); it++){
        map[it->first]->run();
    }



///XXXXXXXXXXXXXXXXXXXXXXXX
    ros::AsyncSpinner spinner(4); // Use 4 threads
//    while (ros::ok()){
//        ros::spinOnce();
//        usleep(10);
//    }

    spinner.start();
    ros::waitForShutdown();
}

int wrapper_get_node_id(){

    return node_id;
}

