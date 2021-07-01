import os 
import time
import xmltodict
import json

xml_file = os.path.join('/', 'cyclonedds.xml')

while True:
    with open(xml_file, 'r') as fd:
        raw_xml = fd.read()
        fd.close()
        
    doc = xmltodict.parse(raw_xml)
    
    myjson = json.loads(json.dumps(doc, indent=2))
    
    myjson['CycloneDDS']['Domain']['Discovery']['Peers'] = {'Peer':[]}
    
    with open('/etc/hosts','r') as hosts_file:
        for line in hosts_file:
            if(line.split()[2:6] == ['#', 'managed', 'by', 'Husarnet']):
                myjson['CycloneDDS']['Domain']['Discovery']['Peers']['Peer'].append({'@address': '[' + line.split()[0] + ']'})
                print('Husarnet IPv6: ' + str(line.split()[0]) + ' | Hostname: ' + str(line.split()[1]))
        hosts_file.close()
            
    print(json.dumps(myjson, indent=2))
    
    new_raw_xml = xmltodict.unparse(myjson, pretty=True)

    with open(xml_file, 'w') as fd:
        fd.write(new_raw_xml)
        fd.close()
    
    time.sleep(5)