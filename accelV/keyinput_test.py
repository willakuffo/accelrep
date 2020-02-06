# testing script

import keyboard
import time
import bluetooth

throttle = ''
throttle_mode = ''
bd_addr = '98:D3:41:FD:6D:F9' #HC-06 address
Buffering = ''
pwm = 0
max_pwm = 1500
min_pwm = 0
port = 1
delay_time = 0.005#current optimal time fs= 1/0.05s = 20Hz  
'''at this period, time to max throttle is about 9s'''
print('Connecting ...')
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))
print('Connection Successful!!!\nConnected to HC-06  on',bd_addr)
time.sleep(5)

'''reducing number of repeatitions per number which is sent would improve responsiveness'''
'''write an algorithm that finds the number of repeatitions per number and optimize to one.
At 0.05s period, fs = 1/0.05 = 20Hz  (time b/n sending each character), this seems optimal'''
while True:
    if keyboard.is_pressed('up'):
        throttle = '1'#+= 1
        throttle_mode = '+UP+'
        Buffering = 'YES'
        pwm += 1 
        if pwm>max_pwm:pwm = max_pwm
        if pwm<min_pwm:pwm = min_pwm
        sock.send(bytes(throttle,'utf-8'))
        
    if keyboard.is_pressed('down'):
        throttle = '0'#-= 1
        throttle_mode = '-DOWN-'
        Buffering = 'NO'
        pwm -= 1
        if pwm>max_pwm:pwm = max_pwm
        if pwm<min_pwm:pwm = min_pwm
        sock.send(bytes(throttle,'utf-8'))
    time.sleep(delay_time)
    print('Data:',throttle,' Throttle Quantity:',pwm,' throttle mode:',throttle_mode,' Buffering:',Buffering)


