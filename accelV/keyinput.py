import keyboard
import time
import bluetooth

throttle = ''
throttle_mode = ''
bd_addr = '98:D3:41:FD:6D:F9' #HC-06 address
Buffering = ''
pwm = 0
port = 1
print('Connecting ...')
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))
print('Connection Successful!!!\nConnected to HC-06  on',bd_addr)
time.sleep(5)


while True:
    if keyboard.is_pressed('up'):
        throttle = '1'#+= 1
        throttle_mode = '+UP+'
        Buffering = 'YES'
        pwm += 1 
        sock.send(bytes(throttle,'utf-8'))
        
    if keyboard.is_pressed('down'):
        throttle = '0'#-= 1
        throttle_mode = '-DOWN-'
        Buffering = 'NO'
        pwm -= 1
        sock.send(bytes(throttle,'utf-8'))
    
    print('Data:',throttle,' Throttle Quantity:',pwm,' throttle mode:',throttle_mode,' Buffering:',Buffering)


