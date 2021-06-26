import time
print("Hello World!")


while True:
    time.sleep(2)
    hosts = open('/etc/hosts','r')
    for line in hosts:
        print(line.split()[1:])