import socket
import json
 
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('localhost', 1234))
print("Connected to socket!") # Use the same unique socket name as in the Qt app
object_data = {
    'objects': [
        {'class': 'triangle', 'x': 100, 'y': 200},
        {'class': 'square', 'x': 300, 'y': 400},
        {'class': 'circle', 'x': 500, 'y': 600},
    ]
}
json_data = json.dumps(object_data)
print(json_data)
client_socket.sendall(json_data.encode('utf-8'))
client_socket.close()

