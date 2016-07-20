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

#include <unistd.h>
#include <stdio.h>
#include <ros/ros.h>
#include <thread>
#include <semaphore.h>
#include <pthread.h>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

//#include "../libwrp/core/queues/include/queues.h"

#define MESSAGE_SIZE 1400

struct TXElem{
    int part;
    int size;
    int message_size;
    int uid;
    unsigned char priority;
    unsigned char port;
    unsigned char src;
    unsigned int dst;
    unsigned int parts;
    char data[MESSAGE_SIZE];
};


std::vector<TXElem> queue;
pthread_mutex_t mtx_tx = PTHREAD_MUTEX_INITIALIZER;
sem_t sem_tx;
std::thread  * tx_loop_th, * rx_loop_th;
int count = 0;

int _node_id;
int _forward_id;

char base_ip[64];
static int txsockfd, rxsockfd;
static struct sockaddr_in txservaddr, rxservaddr, cliaddr;

static int readllcfg() {
    char filename[256], line[256];
    char param[20], val[20];

    snprintf(base_ip, 20, "%s", "192.168.1.1");
    snprintf(filename, 256, "%s/.rospound/rospound.cfg", getenv("HOME"));

    FILE * f = fopen(filename, "r");
    if (f > 0) {
        fprintf(stderr, "Reading Low Level Configuration file (%s)... \n", filename);
        while (fgets(line, 256, f) != NULL) {

            if (line[0] < 65 || line[0] > 90) {
                continue;
            }
            sscanf(line, "%s %s", param, val);

            int exists = 0;
            if (strcmp(param, "BASE_IP") == 0) {
                strcpy(base_ip, val);
                exists = 1;
            }

            if (exists) {
                fprintf(stderr, "READ OPTION: %s = %s\n", param, val);
            } else {
                fprintf(stderr, "*** UKNOWN OPTION: %s = %s\n", param, val);
            }
        }
        fprintf(stderr, "Done.\n");
    } else {
        fprintf(stderr, "File %s not found, using default values\n", filename);
    }

    struct in_addr addr;
    addr.s_addr = htonl(ntohl(inet_addr(base_ip)) + 1);
    char *dot = inet_ntoa(addr);
    fprintf(stderr, "Node one has IP: %s\n", base_ip);
    fprintf(stderr, "Node two has IP: %s, etc.\n", dot);

    return 0;
}

void tx_loop(){
    while (1){
        sem_wait(&sem_tx);
        pthread_mutex_lock(&mtx_tx);
        int max_priority = -1;
        int max_priority_id = -1;
        for (int i = 0; i< queue.size(); i++){
            if (queue.at(i).priority > max_priority){
                max_priority = queue.at(i).priority;
                max_priority_id = i;
            }
        }
        if (max_priority_id >= 0){
            TXElem & txe = queue.at(max_priority_id);

            int address = htonl(ntohl(inet_addr(base_ip)) + _forward_id);

            txservaddr.sin_addr.s_addr = address;
            txservaddr.sin_port = htons(32000);
            sendto(txsockfd, &txe, sizeof(txe) - MESSAGE_SIZE + txe.size, 0, (struct sockaddr *) &txservaddr, sizeof(txservaddr));
            count ++;
            queue.erase(queue.begin() + max_priority_id);
        }
        pthread_mutex_unlock(&mtx_tx);
    }
}

void rx_loop(){
    char bufrx[MESSAGE_SIZE];
    TXElem * txe = (TXElem *) bufrx;
    while (1){

        int len = sizeof(cliaddr);
        int size = recvfrom(rxsockfd, bufrx, MESSAGE_SIZE, 0, (struct sockaddr *) &cliaddr, (socklen_t *)&len);

        pthread_mutex_lock(&mtx_tx);
        queue.push_back(*txe);
        pthread_mutex_unlock(&mtx_tx);
        sem_post(&sem_tx);
    }
}

int main(int argc, char * argv[]){

    if (argc != 3){
        fprintf(stderr,"Usage: %s node_id forward_id\n", argv[0]);
        exit(0);
    }

    _node_id = atoi(argv[1]);
    _forward_id = atoi(argv[2]);

    readllcfg();
    sem_init(&sem_tx,0,0);

    /*SOCKETS*/
    txsockfd = socket(AF_INET, SOCK_DGRAM, 0);
    bzero(&txservaddr, sizeof(txservaddr));
    txservaddr.sin_family = AF_INET;

    rxsockfd= socket(AF_INET, SOCK_DGRAM, 0);
    bzero(&rxservaddr, sizeof(rxservaddr));
    rxservaddr.sin_family = AF_INET;

    int address = htonl(ntohl(inet_addr(base_ip)) + _node_id);
    rxservaddr.sin_addr.s_addr = address; // htonl(INADDR_ANY);
    rxservaddr.sin_port = htons(32000);
    int res = bind(rxsockfd, (struct sockaddr *) &rxservaddr, sizeof(rxservaddr));
    if (res != 0) {
        fprintf(stderr, "*** ABORTING *** Bind error: do IP address and node-id are coherent in this machine?\n");
        exit(0);
    }
    tx_loop_th = new std::thread(tx_loop);
    rx_loop_th = new std::thread(rx_loop);

    while (true){
        fprintf(stderr,"Routed %d frames queue %d    \r", count, queue.size());
        usleep(10000);
    }


    return 0;
}
