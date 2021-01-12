import socket
import re
import subprocess
import os

NAVIO_HOSTNAME = "navio"

def get_first_three_terms_of_ip():
	ip_string = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
	pattern = re.compile('([0-9]{1,3}.){3}')
	match = pattern.match(ip_string)
	return match.group()

def get_all_active_ips(ip_start):
	nmap_addresses = ip_start+"0-255"
	print("Scanning Network for Addresses: " +nmap_addresses)
	response = subprocess.check_output(["nmap","-sn",nmap_addresses])
	response = str(response)

	#get IPs from nmap output
	pattern = re.compile('[0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}')
	ip_list = re.findall(pattern, response)
	return ip_list

# Get GCS IP
ip_start = get_first_three_terms_of_ip()
available_ips = get_all_active_ips(ip_start)
print("Which IP address is the Ground Control Station?")
for i,ip in enumerate(available_ips):
	print("("+str(i)+")"+ip)
gcs_ip = available_ips[int(input())]

# Set apm.launch
launchfile_path = '/opt/ros/noetic/share/mavros/launch/apm.launch'
my_file = open(launchfile_path, "r")
string_list = my_file.readlines()
my_file.close()
for i,line in enumerate(string_list):
    if line.startswith('\t<arg name="gcs_url"'):
        line = '\t<arg name="gcs_url" default="udp://:14551@'+gcs_ip+':14550" />\n'
        string_list[i] = line
new_file_contents = "".join(string_list)
my_file = open(launchfile_path, "w")
my_file.write(new_file_contents)
my_file.close()

print('Setup complete. Please run "roslaunch mavros apm.launch"')
