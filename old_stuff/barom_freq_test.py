from dronekit import connect, VehicleMode
import time

frequencies = {}




vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
vehicle.flush()
@vehicle.on_message('*')
def _on_message(self, name, msg):
    try:
        frequencies[msg.get_type()]
    except KeyError:
        frequencies[msg.get_type()] = 0
    frequencies[msg.get_type()] += 1
vehicle.mode = VehicleMode("GUIDED_NOGPS")
while not vehicle.mode.name == "GUIDED_NOGPS":
    print("Waiting for mode change...")
    time.sleep(1)
vehicle.armed=True
while not vehicle.armed:
    print('Waiting for arming...Mode: %s'%vehicle.mode)
    time.sleep(2)
print('Is Armed: %s'%vehicle.armed)
# t = time.time()
# t_last_update = t
# last_h = 0
# for i in range(5000):
# 	new_h = vehicle.location.global_relative_frame.alt
# 	if new_h is not last_h:
# 		print("Barom update freq is %.4f Hz"%(1/(time.time()-t_last_update)))
# 		t_last_update = time.time()
# 	last_h = new_h
# 	time.sleep(.01)

vehicle.mode = VehicleMode("LAND")

while not vehicle.mode == "LAND":
    print("Waiting for vehicle to land")
    time.sleep(1)

vehicle.armed = False
while vehicle.armed:
    print("Waiting for vehicle to disarm")
    time.sleep(1)
print(frequencies)
vehicle.mode = VehicleMode("GUIDED_NOGPS")
vehicle.close()
print("Done!")
