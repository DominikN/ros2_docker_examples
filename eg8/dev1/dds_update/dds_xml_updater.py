import os 
import time
import xml.etree.ElementTree as ET

print("Hello World!")

tree = ET.parse(os.path.join('/dds', 'cyclonedds.xml'))
root = tree.getroot()

for peer in root.findall('.//{https://cdds.io/config}Peer'):
    print(peer.tag, peer.attrib)
    print(peer.attrib['address'])
    
while True:
    time.sleep(2)
    
    # remove all Peers from xml
    for peer in root.findall('.//{https://cdds.io/config}Peer'):
        root.remove(peer)
    
    hosts = open('/etc/hosts','r')
    for line in hosts:
        if(line.split()[2:6] == ['#', 'managed', 'by', 'Husarnet']):
            print(line.split()[0])  # Husarnet IPv6 address
            print(line.split()[1])  # Husarnet Hostname
            ET.SubElement('.//{https://cdds.io/config}Peers', 'Peer', address=line.split()[0])