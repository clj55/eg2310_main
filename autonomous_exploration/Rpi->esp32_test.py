import requests
import json

esp32_ip = '172.20.10.5' # this ip changes everytime you use it, but the ip wont change depending on the computer
turtleBot_ID = '43' # turtlebot3 burger ID we receive on the first day

endpoint = "http://" + esp32_ip + "/openDoor" # url to connect to the server of esp32, which then goes into the directory "openDoor"
data = {"action": "openDoor", "parameters": {"robotId": turtleBot_ID}} # the messages the esp32 checks to see if it should open the door
header = {'Content-Type': 'application/json'} # tells the esp32 what type of file we are sending

response = requests.post(endpoint, json=data, headers=header) # writes the output while sends the http request to the esp32

print(response.text) # printing outcome
