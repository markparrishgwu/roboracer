import serial.tools.list_ports

print("enumerating serial devices")
for port in serial.tools.list_ports.comports():
    print(port)