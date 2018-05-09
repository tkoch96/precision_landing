from dronekit import connect, VehicleMode
import time




vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
vehicle.flush()
print('Is Armed: %s\n'%vehicle.armed)

vehicle.mode = VehicleMode("LAND")

while not vehicle.mode == "LAND":
    print("Waiting for vehicle to land")
    time.sleep(1)

vehicle.armed = False
while vehicle.armed:
    print("Waiting for vehicle to disarm")
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
vehicle.close()
print("Done!")
