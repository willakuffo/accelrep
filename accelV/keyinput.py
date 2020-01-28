import keyboard
import time
import bluetooth
throttle = 0
end = 0
thruput = 0

bd_addr = 'B8:27:EB:DC:71:DA' #pis address

port = 4

sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))


while True:
    if keyboard.is_pressed('up'):
        throttle += 1
        
    if keyboard.is_pressed('down'):
        throttle -= 1
        print(throttle)
    if throttle >= 80000:
            print("MAX")
    if throttle <= 0:
            print("MIN")
    thruput = (throttle//20000)*180       
    sock.send(str(thruput))
    print(throttle,thruput)

