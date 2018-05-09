from dronekit import connect, VehicleMode
import time



target_alt = .8
vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
vehicle.flush()
vehicle.mode = VehicleMode("GUIDED_NOGPS")
while not vehicle.mode.name == "GUIDED_NOGPS":
    print("Waiting for mode change...")
    time.sleep(1)
vehicle.armed=True
while not vehicle.armed:
    print('Waiting for arming...Mode: %s'%vehicle.mode)
    time.sleep(2)
print('Is Armed: %s'%vehicle.armed)

print('Taking Off ...')
vehicle.simple_takeoff(target_alt)
i=0
while i<50:
    print('Altitude: ', vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= target_alt:
        print('Reached target altitude..')
        break
    time.sleep(.1)
    i += 1

vehicle.mode = VehicleMode("LAND")

while not vehicle.mode == "LAND":
    print("Waiting for vehicle to land")
    time.sleep(1)

vehicle.armed = False
while vehicle.armed:
    print("Waiting for vehicle to disarm")
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED_NOGPS")
vehicle.close()
print("Done!")
