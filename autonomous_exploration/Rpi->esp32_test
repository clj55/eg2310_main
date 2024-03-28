import requests
esp32_ip = '172.20.10.4'
turtleBot_ID = 43

endpoint = "http://" + esp32_ip + "/openDoor"
data = {"action": "openDoor", "parameters": {"robotId": turtleBot_ID}}
header = {'Content-Type': 'application/json'}

requests.post(endpoint, params=data, headers=header)
response = requests.get(endpoint, params=data, headers=header)

print(response)
