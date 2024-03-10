#!/bin/bash
ROBOT_IP="192.168.123.161"
PORTS="1:65535"

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward

# Flush existing rules
sudo iptables -F
sudo iptables -t nat -F

# Set default policies
sudo iptables -P INPUT ACCEPT
sudo iptables -P FORWARD ACCEPT
sudo iptables -P OUTPUT ACCEPT

# Apply NAT (MASQUERADE) to packets leaving wlan0 interface
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE

# Apply NAT (MASQUERADE) to packets leaving eth0 interface
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE


# Allow related and established connections
sudo iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT

# Allow forwarding for TCP traffic
sudo iptables -t nat -A PREROUTING -p tcp -i wlan0 --dport $PORTS -j DNAT --to-destination $ROBOT_IP

# Allow forwarding for UDP traffic
sudo iptables -t nat -A PREROUTING -p udp -i wlan0 --dport $PORTS -j DNAT --to-destination $ROBOT_IP

# Allow incoming UDP traffic from the robot in response to the remote PC
sudo iptables -t nat -A POSTROUTING -p udp -d $ROBOT_IP --sport $PORTS -j SNAT --to-source 10.5.98.70



# Allow forwarding of packets from wlan0 to eth0
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT

# Allow forwarding of packets from eth0 to wlan0
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT


