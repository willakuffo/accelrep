import bluetooth

def server():
    
    server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

    port = 4
    server_sock.bind(("",port))
    server_sock.listen(5)

    client_sock,address = server_sock.accept()
    print ("Accepted connection from ",address)

    data = client_sock.recv(1024)
    print ("received [%s]" % data)

    client_sock.close()
    server_sock.close()

def client():
    bd_addr = '98:D3:41:FD:6D:F9' #HC-06 address

    port = 4

    sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
    sock.connect((bd_addr, port))
    while True:
        sock.send(str(data))

    #sock.close()
client()
