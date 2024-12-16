from time import sleep

import PythonAPI

api = PythonAPI.uavAPI()

for i in range(10):
    data = api.getSensorData()
    print(data.position)
    print(data.timestamp)
    sleep(0.1)

