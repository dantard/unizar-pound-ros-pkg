
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

#include<ros/ros.h>
#include<yaml-cpp/yaml.h>

#define ROSWMP_DEBUG(output,...)  //  fprintf(output, __VA_ARGS__); fprintf(output,"\n");


int wrapper_get_node_id();

int wrapper_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned int dst, unsigned int deadline);

void * wrapper_pop(int port, int timeout, char* & data, unsigned int & size, unsigned char & src, unsigned char & priority);

int wrapper_pop_done(void * data);

/* needed by QoS topics */
int wrapper_get_num_of_enqueued_elements(int port);

int wrapper_init(ros::NodeHandle & n, int node_id, int num_of_nodes, int num_of_ports);

void wrapper_run_bg();

int wrapper_flow_add(unsigned int port, unsigned int period, unsigned short priority, unsigned int deadline);

int wrapper_set_period(int port, int period);
int wrapper_set_priority(int port, int priority);
bool wrapper_call_service(std::string command, std::string param1, int param2, std::string & info, int &result);

std::string wrapper_get_config_filename();
void wrapper_setup_config(YAML::Node, int _node_id, int _num_of_nodes, int _num_of_ports);
int wrapper_main(int argc, char * argv[]);
