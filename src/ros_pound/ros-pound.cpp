
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

/* the L2 protocols */
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sstream>

#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <linux/if_link.h>

#include <yaml-cpp/yaml.h>

#define MESSAGE_SIZE 1400
#define MAX_HOPS 5
#define MAX_80211_FRAME 2342

int wrapper_main(int argc, char * argv[]);


struct TXElem{
    int part;
    int size;
    int message_size;
    int uid;
    unsigned int deadline;
    unsigned int timestamp;
    unsigned char priority;
    unsigned char port;
    unsigned char src;
    unsigned int dst;
    unsigned int parts;
    char data[MESSAGE_SIZE];
};

struct RXElem{
    std::vector<unsigned char> parts;
    int message_size;
    unsigned char priority;
    unsigned char port;
    unsigned char src;
    unsigned int dst;
    int uid;
    std::vector<char> data;
};

struct mac_t{
    unsigned char mac[6];
    int valid;
    mac_t(){
        valid = 0;
    }
};

sem_t sem_tx;
pthread_mutex_t mtx_tx = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mtx_rx = PTHREAD_MUTEX_INITIALIZER;
std::thread  * tx_loop_th, * rx_loop_th, * st_loop_th;

std::vector<TXElem> queue, unconfirmed;
std::vector<sem_t> sems_rx;
std::vector< std::vector<RXElem> > queues_rx;
std::vector< int > gateway;
std::vector<mac_t> nodes;
std::map< std::pair <int, int>, RXElem > map_rx;

int uid = 0, queue_size = 50;
int txi  = 0;
int use_ip = 1, use_discard = 0, delay = 2500;
int port = 32000;
int txsockfd, rxsockfd;
int node_id, num_of_nodes, auto_tuning = 0, feedback = 0, quiet = 0;
int discarded = 0, forwarded = 0, sent = 0, enqueued = 0, received = 0, byte_sent = 0;

int should_discard = 0;


char base_ip[64]="192.168.1.1";
char device[64]="wlan0";
unsigned char tx_mac[6];

struct sockaddr_in txservaddr, rxservaddr, cliaddr;

int receive_raw(char * buf){

    static char rxbuf[2342];

    int rlen = recvfrom(rxsockfd, rxbuf, 2342, 0, 0, 0);
    struct ethhdr * eh = (struct ethhdr *) rxbuf;

    if (eh->h_proto == 0x6464){
        rlen = rlen - ETH_HLEN;
        memcpy(buf, rxbuf + ETH_HLEN, rlen);
    }else{
        rlen = -1;
    }
    return rlen;
}

void send_raw(int dest, char * data, int len){

    static char txb[2342];
    static struct ethhdr * eh = (struct ethhdr *) txb;
    static struct sockaddr_ll broadcast;
    static unsigned char bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    if (nodes[dest].valid){
        memcpy((void *) eh->h_dest, (void*) nodes[dest].mac, ETH_ALEN);
        broadcast.sll_pkttype = PACKET_HOST;
    } else {
        memcpy((void *) eh->h_dest, (void*) bcast_mac, ETH_ALEN);
        broadcast.sll_pkttype = PACKET_BROADCAST;
    }

    broadcast.sll_family = PF_PACKET;
    broadcast.sll_protocol = htons(0x6464);
    broadcast.sll_ifindex = txi;
    broadcast.sll_halen = ETH_ALEN;

    eh->h_proto = htons(0x6464);
    memcpy((void *) eh->h_source, (void*) tx_mac, ETH_ALEN);
    memcpy(txb + ETH_HLEN, data, len);

    sendto(txsockfd, txb, len + ETHER_HDR_LEN, 0, (struct sockaddr*) &broadcast, sizeof(broadcast));

}

static int sock_raw_init(const char * DEVICE, int protocol, int * sock, int * if_idx, unsigned char src_mac[6]) {
    int s, ifindex, i;
    struct ifreq ifr;

    s = socket(PF_PACKET, SOCK_RAW, htons(protocol));
    if (s == -1) {
        perror("socket():");
        return 1;
    }

    strncpy(ifr.ifr_name, DEVICE, IFNAMSIZ);
    if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
        perror("SIOCGIFINDEX");
        return 1;
    }
    ifindex = ifr.ifr_ifindex;

    if (ioctl(s, SIOCGIFHWADDR, &ifr) == -1) {
        perror("SIOCGIFINDEX");
        return 1;
    }

    for (i = 0; i < 6; i++) {
        src_mac[i] = ifr.ifr_hwaddr.sa_data[i];
    }

    /* bind socket to interface to receive frame ONLY from that interface */
    struct sockaddr_ll sll;
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_protocol = htons(protocol);
    if ((bind(s, (struct sockaddr *) &sll, sizeof(sll))) == -1) {
        perror("bind: ");
        return 1;
    }

    (*sock) = s;
    (*if_idx) = ifindex;
    return 0;
}

int get_name_from_ip (){
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) {
        return -1;
    }
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL){
            continue;
        }
        struct sockaddr_in * sa = (struct sockaddr_in *)(ifa->ifa_addr);
        if (sa->sin_addr.s_addr == rxservaddr.sin_addr.s_addr){
            sprintf(device,"%s", ifa->ifa_name);
            return 0;
        }
    }
    return -2;
}

int get_sent_packets(){
    struct ifaddrs *ifaddr, *ifa;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return -1;
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL){
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_PACKET && ifa->ifa_data != NULL && strcmp(ifa->ifa_name, device) == 0){
            struct rtnl_link_stats *stats = (struct rtnl_link_stats *)ifa->ifa_data;
            int txp = stats->tx_packets;
            freeifaddrs(ifaddr);
            return txp;
        }
    }
    return -2;
}


unsigned long long int get_timestamp_us(){
    struct timeval tp;
    static unsigned long long t_zero = 0;
    gettimeofday(&tp, NULL);
    unsigned long long mslong = (long long) tp.tv_sec * 1000000L + tp.tv_usec; //get current timestamp in milliseconds
    if (t_zero == 0){
        t_zero = mslong;
    }
    return (mslong - t_zero);
}

unsigned int get_timestamp(){
    struct timeval tp;
    static unsigned long long t_zero = 0;
    gettimeofday(&tp, NULL);
    unsigned long long mslong = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000; //get current timestamp in milliseconds
    if (t_zero == 0){
        t_zero = mslong;
    }
    return (int) (mslong - t_zero);
}


int mcast_to_id(unsigned int dest){
    int ip = 0;
    while (dest >>= 1) {
        ++ip;
    }
    return ip;
}

static std::vector<int> mcast_to_vector(unsigned int dest){
    std::vector<int> v;
    for (int i = 0; i<32; i++){
        if (dest & (1 << i)){
            v.push_back(i);
        }
    }
    return v;
}


std::string wrapper_get_config_filename(){
    return std::string(".rospound/config.yaml");
}

void wrapper_setup_config(YAML::Node yaml, int _node_id, int _num_of_nodes, int _num_of_ports){
    if (YAML::Node parameter = yaml["use_ip"]) {
        use_ip = parameter.as<bool>();
        std::cout << " Setting: use_ip = " << (use_ip?"true":"false") << std::endl;
    }
    if (YAML::Node parameter = yaml["use_discard"]) {
        use_discard = parameter.as<bool>();
        std::cout << " Setting: use_discard = " << (use_discard?"true":"false") << std::endl;
    }
    if (YAML::Node parameter = yaml["rate_mbps"]) {
        float rate = parameter.as<float>()*1e6*0.7834;
        float real_rate = rate/float(_num_of_nodes - 1);
        float f_delay = 1e6/(real_rate/(1500.0*8.0))*0.93;
        delay = (int) f_delay;
        std::cout << " Setting: rate_mbps = " << rate << std::endl;
    }
    if (YAML::Node parameter = yaml["delay"]) {
        delay = parameter.as<int>();
        std::cout << " Setting: delay = " << delay << std::endl;
    }
    if (YAML::Node parameter = yaml["queue"]) {
        queue_size = parameter.as<int>();
        std::cout << " Setting: queue = " << queue_size << std::endl;
    }
    if (YAML::Node parameter = yaml["quiet"]) {
        quiet = parameter.as<bool>();
        std::cout << " Setting: quiet = " << quiet << std::endl;
    }
    if (YAML::Node parameter = yaml["feedback"]) {
        feedback = parameter.as<bool>();
        std::cout << " Setting: feedback = " << feedback << std::endl;
        if (YAML::Node parameter = yaml["auto_tuning"]) {
            auto_tuning = parameter.as<bool>();
            std::cout << " Setting: auto_tuning = " << auto_tuning << std::endl;
        }
    }

    if (use_ip){
        if (YAML::Node parameter = yaml["base_ip"]) {
            sprintf(base_ip, "%s", parameter.as<std::string>().c_str());
            struct in_addr addr;
            addr.s_addr = htonl(ntohl(inet_addr(base_ip)) + 0);
            char *check = inet_ntoa(addr);
            std::cout << " Setting: base_ip = " << check << std::endl;
        }
        if (YAML::Node parameter = yaml["port"]) {
            port = parameter.as<int>();
            std::cout << " Setting: port = " << port << std::endl;
        }
    } else {
        if (YAML::Node parameter = yaml["device"]) {
            sprintf(device, "%s", parameter.as<std::string>().c_str());
            std::cout << " Setting: device = " << device << std::endl;
        }
        if (YAML::Node parameter = yaml["nodes"]) {
            std::cout << " Setting: nodes -> " << std::endl;
            for (int i = 0; i< parameter.size(); i++){

                YAML::Node node = parameter[i];

                int id = node["id"].as<int>();
                std::string mac_str = node["mac"].as<std::string>().c_str();

                if (nodes.size() < id + 1){
                    nodes.resize(id + 1);
                }

                mac_t mac;
                mac.valid = 1;
                unsigned int uimac[6];

                sscanf(mac_str.c_str(),"%x:%x:%x:%x:%x:%x",&uimac[0], &uimac[1],&uimac[2], &uimac[3],&uimac[4], &uimac[5]);
                for (int i=0; i<6; i++) mac.mac[i] = (unsigned char) uimac[i];
                nodes[id] = mac;
                fprintf(stderr,"  Node %d = %02x:%02x:%02x:%02x:%02x:%02x\n", id, mac.mac[0], mac.mac[1],mac.mac[2], mac.mac[3],mac.mac[4], mac.mac[5]);
            }
        }
    }

    if (YAML::Node parameter = yaml["routes"]) {
        std::cout << " Setting: routes -> " << std::endl;
        for (int i = 0; i< parameter.size(); i++){
            YAML::Node gw = parameter[i];
            int dest = gw["dest"].as<int>();
            int next = gw["next"].as<int>();
            if (gateway.size() < dest + 1){
                gateway.resize(dest + 1, -1);
            }
            gateway[dest] = next;
            std::cout << "  For node "<< dest << " next hop is "<< next << std::endl;
        }
    }
    /*
        use_ip: true
        device: wlan0
        base_ip: 192.168.1.1
        nodes:
         - id: 3
           mac: 22:44:55:FF:00:01
         - id: 2
           mac: 18:19:20:21:22:23
        routes:
         - dest: 3
           next: 5
         - dest: 4
           next: 5
        feedback: false
        auto_tuning: false
        port: 32000
        queue: 50
        delay: 2500
        rate_mbps: 6.0
        use_discard: false
        quiet: false
    */

}
int wrapper_init(ros::NodeHandle & n, int _node_id, int _num_of_nodes, int _num_of_ports){
    std::cerr << "Checking timer...";
    get_timestamp();
    usleep(100000);
    std::cerr << "Are these -> "<< get_timestamp() << " <- 100 ms?"<< std::endl;

    node_id = _node_id;
    num_of_nodes = _num_of_nodes;

    sem_init(&sem_tx,0,0);
    sems_rx.resize(_num_of_ports);

    for (int i  = 0; i< _num_of_ports; i++){
        sem_init(&sems_rx[i], 0, 0);
    }
    queues_rx.resize(_num_of_ports);

    if (use_ip){
        /*SOCKETS*/
        txsockfd = socket(AF_INET, SOCK_DGRAM, 0);
        bzero(&txservaddr, sizeof(txservaddr));
        txservaddr.sin_family = AF_INET;

        rxsockfd= socket(AF_INET, SOCK_DGRAM, 0);
        bzero(&rxservaddr, sizeof(rxservaddr));
        rxservaddr.sin_family = AF_INET;

        int address = htonl(ntohl(inet_addr(base_ip)) + _node_id);
        rxservaddr.sin_addr.s_addr = address; // htonl(INADDR_ANY);
        rxservaddr.sin_port = htons(port);
        int res = bind(rxsockfd, (struct sockaddr *) &rxservaddr, sizeof(rxservaddr));
        if (res != 0) {
            fprintf(stderr, "*** ABORTING *** Bind error: do IP address and node-id are coherent in this machine?\n");
            exit(0);
        }
        get_name_from_ip();
    }else{
        if (sock_raw_init(device, 0x6464, & txsockfd, &txi, tx_mac)){
            exit(1);
        }
        rxsockfd = txsockfd;
    }

    return 0;
}

void here(int id, int val= -1){
    static int last = 0;
    static int last_id = 0;
    unsigned long long int now = get_timestamp_us();
    unsigned long long int elapsed = now - last;

    if (val != -1 ){
        fprintf(stderr,"Between %3d and %3d: %5lld us -> %5d\n", last_id, id, elapsed, val);
    }else{
        fprintf(stderr,"Between %3d and %3d: %lld us\n", last_id, id, elapsed);
    }
    last = now;
    last_id = id;
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

            int next_hop = txe.dst;
            if (txe.dst < gateway.size() && gateway[txe.dst] != -1){
                next_hop = gateway[txe.dst];
            }

            int age = get_timestamp() - txe.timestamp;
            if (1){
                //if ( (!use_discard) || txe.deadline == 0 || age < txe.deadline){
                int address = htonl(ntohl(inet_addr(base_ip)) + next_hop);
                txservaddr.sin_addr.s_addr = address;
                txservaddr.sin_port = htons(port);
                txe.deadline = txe.deadline - age;
                int size = sizeof(txe) - MESSAGE_SIZE + txe.size;

                if (use_ip){
                    if (sendto(txsockfd, &txe, size, 0, (struct sockaddr *) &txservaddr, sizeof(txservaddr)) != size){
                        perror("Unable to send packet of this size...");
                    }
                }else{
                    send_raw(next_hop, (char*) &txe, size);
                }

                byte_sent += size;
                sent ++;
                queue.erase(queue.begin() + max_priority_id);
                pthread_mutex_unlock(&mtx_tx);

                /* TIME NEEDED TO SEND THE PACKET */
                usleep(delay);
            }else{
                discarded ++ ;
                queue.erase(queue.begin() + max_priority_id);
                pthread_mutex_unlock(&mtx_tx);
            }
        }else{
            pthread_mutex_unlock(&mtx_tx);
        }
    }
}

void rx_loop(){
    char bufrx[MAX_80211_FRAME];
    TXElem * txe = (TXElem *) bufrx;
    while (1){

        int size;
        if (use_ip){
            socklen_t len = sizeof(cliaddr);
            size = recvfrom(rxsockfd, bufrx, MAX_80211_FRAME, 0, (struct sockaddr *) &cliaddr, &len);
        }else{
            size = receive_raw(bufrx);
        }

        if (size < 0){
            continue;
        }

        //      fprintf(stderr,"RECEIVING PARTs %d SIZE:%d from %d to %d\n", 0, size, txe->src, txe->dst);

        received ++;

        /* NOT FOR ME -> FORWARDING */
        if (txe->dst != node_id){
            pthread_mutex_lock(&mtx_tx);
            txe->timestamp = get_timestamp();
            queue.push_back(*txe);
            pthread_mutex_unlock(&mtx_tx);
            sem_post(&sem_tx);
            forwarded ++;
            continue;
        }

        pthread_mutex_lock(&mtx_rx);

        std::pair<int,int> key(txe->src, txe->uid);
        RXElem & rxe = map_rx[key];

        if (rxe.parts.size() == 0){
            rxe.parts.resize(txe->parts,0);
        }
        if (rxe.data.size() == 0){
            rxe.data.resize(txe->message_size);
        }
        rxe.port = txe->port;
        rxe.priority = txe->priority;
        rxe.src = txe->src;
        rxe.message_size = txe->message_size;
        rxe.uid = txe->uid;

        memcpy(&rxe.data[MESSAGE_SIZE*txe->part], txe->data,txe->size);

        rxe.parts[txe->part] = 1;

        int sum = 0;
        for (int i = 0; i<rxe.parts.size(); i++){
            sum += rxe.parts[i];
        }

        if (txe->port >= queues_rx.size()){
            ROS_ERROR("*** WARNING: Received packet for undefined port...aborting.");
            ROS_ERROR("Have you specified the same number of nodes in all the instances?");
            ROS_ERROR("Have you the same config file in all the instances?");
            exit(0);
        }

        if (sum == rxe.parts.size()){
            queues_rx[rxe.port].push_back(rxe);
            sem_post(&sems_rx[rxe.port]);
            map_rx.erase(key);
        }
        pthread_mutex_unlock(&mtx_rx);

    }
}

void st_loop(){
    int last = 0, count = 0, count2 = 0;
    while (ros::ok()){

        if (feedback){
            int ifsent = get_sent_packets();
            if (ifsent >= 0){

                int diff = ifsent - sent;
                should_discard = abs(diff-last) > 2;
                last = diff;

                if (auto_tuning){
                    if (should_discard){
                        count2++;
                        if (count2 > 2){
                            delay += 50;
                            count = 0;
                            count2 = 0;
                        }
                    }else{
                        count++;
                        if (count > 100){
                            delay-=25;
                            count = 0;
                            count2 = 0;
                        }
                    }
                }
            }
        }
        if (!quiet){
            static unsigned long long prev_ts = get_timestamp_us();
            static int prev_byte = byte_sent;
            static double bw = 0;

            unsigned long long now_ts = get_timestamp_us();
            int now_byte = byte_sent;

            unsigned long long elapsed = now_ts - prev_ts;

            if (elapsed > 1e6){
                bw = 1000.0*double(now_byte - prev_byte)*8.0/double(elapsed);
                prev_ts = now_ts;
                prev_byte = now_byte;
            }

            fprintf(stderr,"Stats: Kbps:%8.2f RX:%6d TX:%6d FW:%6d DS:%6d || ENQ:%6d TXQ:%6d MAP:%3d DLY:%4d SD:%d Delay:%5d\r",
                    bw,
                    received,
                    sent,
                    forwarded,
                    discarded,
                    enqueued,
                    (int) queue.size(),
                    (int) map_rx.size(),
                    queue.size()> 0 ? get_timestamp() - queue.at(0).timestamp: 0,
                    should_discard,
                    delay);
        }
        usleep(250000);
    }
}

void wrapper_run_bg(){
    tx_loop_th = new std::thread(tx_loop);
    rx_loop_th = new std::thread(rx_loop);
    st_loop_th = new std::thread(st_loop);
}

int wrapper_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned int dst, unsigned int deadline){

    std::vector<int> dests = mcast_to_vector(dst);

    if (size > 10000 && (should_discard || queue.size() > queue_size)){
        discarded ++;
        return 0;
    }

    /* Local Delivery */
    auto it = std::find(dests.begin(), dests.end(), node_id);
    if (it != dests.end()){
        RXElem e;
        e.src = src;
        e.priority =priority;
        e.message_size = size;
        e.data.resize(size);
        e.port = port;
        memcpy(&e.data[0], data, size);
        pthread_mutex_lock(&mtx_rx);
        queues_rx[e.port].push_back(e);
        pthread_mutex_unlock(&mtx_rx);
        sem_post(&sems_rx[e.port]);
        dests.erase(it);
        if (dests.size() == 0){
            return 0;
        }
    }

    TXElem txe;
    int parts = ceil(double(size) / double(MESSAGE_SIZE)), count = 0;
    pthread_mutex_lock(&mtx_tx);
    uid = uid + 1;
    for (int i = 0; i < parts; i++){
        txe.parts = parts;
        txe.part = i;
        txe.size = MESSAGE_SIZE * (i+1) < size? MESSAGE_SIZE : size - MESSAGE_SIZE * i;
        txe.priority = priority;
        txe.src = src;
        txe.port = port;
        txe.uid = uid;
        txe.message_size = size;
        txe.timestamp = get_timestamp();
        txe.deadline = deadline;
        memcpy(&txe.data[0], &data[MESSAGE_SIZE * i], txe.size);

        for (int i = 0; i< dests.size(); i++){
            txe.dst = dests.at(i);
            queue.push_back(txe);
            count ++;
        }
    }
    pthread_mutex_unlock(&mtx_tx);

    for (int i = 0; i< count; i++){
        sem_post(&sem_tx);
        usleep(10);
        enqueued ++;
    }

    return 0;
}

void * wrapper_pop(int port, int timeout, char* & data, unsigned int & size, unsigned char & src, unsigned char & priority){
    sem_wait(&sems_rx[port]);
    pthread_mutex_lock(&mtx_rx);
    RXElem & e = queues_rx[port].at(0);
    size = e.message_size;
    src = e.src;
    priority = e.priority;
    data = &e.data[0];
    return (void *) new std::pair <int, int> (port,0);
}


int wrapper_pop_done(void * data){
    std::pair<int,int> * p = (std::pair<int,int> *) data;
    queues_rx[p->first].erase(queues_rx[p->first].begin()+p->second);
    pthread_mutex_unlock(&mtx_rx);

    delete p;
    return 0;
}

int wrapper_get_num_of_enqueued_elements(int port){
    pthread_mutex_lock(&mtx_rx);
    int res = queues_rx[port].size();
    pthread_mutex_unlock(&mtx_rx);
    return res;
}


int wrapper_flow_add(unsigned int port, unsigned int period, unsigned short priority, unsigned int deadline){
    return 0;
}

int wrapper_set_period(int port, int period){
    return 0;
}

int wrapper_set_priority(int port, int priority){
    return 0;
}
bool wrapper_call_service(std::string command, std::string param1, int param2, std::string & info, int &result){
    return false;
}

//int main(int argc, char * argv[]){
//    return wrapper_main(argc, argv);
//}
