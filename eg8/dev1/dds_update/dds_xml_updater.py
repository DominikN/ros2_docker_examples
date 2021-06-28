import os 
import time
import xml.etree.ElementTree as ET

print("Hello World!")
xml_file = os.path.join('/dds', 'cyclonedds.xml')
ET.register_namespace("", "https://cdds.io/config")

tree = ET.parse(xml_file)
root = tree.getroot()

for peer in root.findall('.//{https://cdds.io/config}Peer'):
    print(peer.tag, peer.attrib)
    print(peer.attrib['address'])
    
while True:
    ET.dump(root)
    
    # remove all Peers from xml
    for peer in root.findall('.//{https://cdds.io/config}Peer'):
        print("peer to remove:")
        print(peer.tag, peer.attrib)
        root.find('.//{https://cdds.io/config}Peers').remove(peer)
    
    hosts = open('/etc/hosts','r')
    for line in hosts:
        if(line.split()[2:6] == ['#', 'managed', 'by', 'Husarnet']):
            print(line.split()[0])  # Husarnet IPv6 address
            print(line.split()[1])  # Husarnet Hostname
            ET.SubElement(root.find('.//{https://cdds.io/config}Peers'), 'Peer', address=line.split()[0])
            with open(xml_file, 'w') as f:
                tree.write(f, encoding='unicode')
    time.sleep(5)