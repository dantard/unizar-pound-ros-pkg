Configuration file to be put in ~/.rospound/config.yaml

Example configuration file:

nodes:
 - id: 3
   mac: 22:44:55:FF:00:01
 - id: 2
   mac: 18:19:20:21:22:23
device: wlan0
use_ip: true
base_ip: 192.168.1.1
routes:
 - dest: 3
   next: 5
 - dest: 4
   next: 5

topics:
 - name: laser
   priority: 45
   period: 500
   source: 0
   dest: 0
   deadline: 45



Topic definition (compile time) is as follows:

TOPIC(type, topic, source, dest, priority, period, time_to_live)

and services as:

SERVICE(type, topic, source, priority, time_to_live)
