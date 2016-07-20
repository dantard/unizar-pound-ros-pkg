# unizar-pound-ros-pkg
The Pound ROS node

Usage:
`rosrun ros_pound ros-pound --node-id 0 --num-of-nodes 3`

Parameter `node-id` specifies the identifier of the local node, parameter `num-of-nodes` specifies the total number of nodos in the robotic network

By default, ros-pound uses UDP/IP communication. The network must have an address of the type `192.168.XX.YY`. XX is the network identificator while YY is related with the `node-id` parameter. Suppose the network is `192.168.1.YY`:

* Node with `--node-id 0` *must* have IP `192.168.1.1`
* Node with `--node-id 1` *must* have IP `192.168.1.2`

etc.

The base IP is by default `192.168.1.1`; this can be changed in the YAML configuration file specifying the parameter:

`base_ip = 192.168.2.1`

for example.

The topics to be transported must be specified in the `config/config.h` file with the format:

`TOPIC(type, topic_name, source, dest, priority, period, time_to_live)`

where

* `type` specifies the data type for example `std_msgs::Float64` 
* `topic_name` specifies the name of the topic, for example `"float_number"` 
* `source` is the source node of the topic, for example `0` 
* `dest` specifies the destination node(s) as text for example `"0,1,2"` however for the moment Pound only support unicast (only one node should be specified) 
* `priority` specifies the priority associated to the topic for example `56`. Must be between 0 and 127 
* `period` the expected period of the topic in ms, for example `100` 
* `time_to_live` period during the which the message is considered valid in ms, for example `500` 

For additional informations contact `dantard@unizar.es`



