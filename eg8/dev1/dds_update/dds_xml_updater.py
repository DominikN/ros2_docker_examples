import os 
import time
import xml.etree.ElementTree as ET
import xmltodict
import json
import pprint
# from collections import OrderedDict

pp = pprint.PrettyPrinter(indent=4)
print("Hello World!")
xml_file = os.path.join('/dds', 'cyclonedds.xml')

while True:
    with open(xml_file, "r") as fd:
        raw_xml = fd.read()
        doc = xmltodict.parse(raw_xml)
        myjson = json.loads(json.dumps(doc))
        fd.close()
    
    print('\r\n[0] Peers:\r\n')
    pp.pprint(raw_xml)
    
    print('\r\n[1] Peers:\r\n')
    pp.pprint(myjson)
    
    print('\r\n[2] Peers:\r\n')
    hosts = open('/etc/hosts','r')
    myjson['CycloneDDS']['Domain']['Discovery']['Peers']['Peer'] = []
    for line in hosts:
        if(line.split()[2:6] == ['#', 'managed', 'by', 'Husarnet']):
            print(line.split()[0])  # Husarnet IPv6 address
            print(line.split()[1])  # Husarnet Hostname
            myjson['CycloneDDS']['Domain']['Discovery']['Peers']['Peer'].append({'@address': line.split()[0]})
    pp.pprint(myjson)
    
    print('\r\n[4] Peers:\r\n')
    new_raw_xml = xmltodict.unparse(myjson, pretty=True)

    print(new_raw_xml)
    
    with open(xml_file, "w") as fd:
        fd.write(new_raw_xml)
        fd.close()
    
    time.sleep(5)
    
    # print('\r\nPeers:\r\n')
    # peers = doc['CycloneDDS']['Domain']['Discovery']['Peers']
    # peers['Peer'].append(OrderedDict[("@address", "jakis-ipv6")])
    # # peers['Peer']['@address']=123
    # print(peers)
    
    # print('\r\nPeers:\r\n')
    # del peers['Peer']
    # print(peers)
    
    # print('number of peers: ' + str(len(doc['CycloneDDS']['Domain']['Discovery']['Peers']['Peer'])))
    # time.sleep(5)

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