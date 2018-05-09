import math, time
import numpy as np
from dronekit import VehicleMode, connect
from MyCopter import MyCopter
import time

#simple tests to check to see if param set works

def main():
	vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
	time.sleep(2)
	#perform pre-arm checks, switch modes
	vehicle.flush()
	cop = MyCopter(vehicle)

if __name__ == "__main__":
	main()