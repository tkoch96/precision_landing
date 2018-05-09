import time, cv2
from dronekit import VehicleMode, connect
from MyCopter import MyCopter
from Image_Processor import Image_Processor
import multiprocessing, threading
from imutils.video import VideoStream, FPS



print("Start timer.")
t_start = time.time()


class Worker_Bee(multiprocessing.Process):
    def __init__(self, task_queue,result_queue,idee):
        multiprocessing.Process.__init__(self)
        self.task_queue = task_queue
        self.result_queue = result_queue
        self.idee = idee

    def run(self):
        proc_name = self.name
        while(True):
            print("Retrieving task on worker %d"%self.idee)
            next_task = self.task_queue.get()
            print("Performing task on worker %d"%self.idee)
            answer = next_task()
            print("Got an answer, adding to result queue.")
            self.task_queue.task_done()
            self.result_queue.put(answer)


def main():
    vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)

    #flight parameters
    go_up = 2
    target_altitude = vehicle.location.global_relative_frame.alt + go_up
    if target_altitude > go_up:
       target_altitude = go_up

    #perform pre-arm checks, switch modes
    vehicle.flush()
    cop = MyCopter(vehicle)

    
    # print("Yaw set, rotate!")
    # time.sleep(10)
    #take off!
    #cop.arm_and_takeoff_nogps(target_altitude)
    # cop.rotate_drone_heading()
    # time.sleep(4)
    #reached target altitude - now land



    print("Switching to land mode")
    vehicle.mode = VehicleMode("LAND")
    
    #main loop
    # lets start our worker bees
    high_res = (1088,720)
    low_res = (320,240)     
    res = high_res
    vs = VideoStream(usePiCamera=True, resolution=res).start()
    time.sleep(1)
    num_workers = 1#multiprocessing.cpu_count()
    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue()
    
    print("Working with %d worker bees."%(num_workers))

    workers = []
    for i in range(num_workers):
        workers.append(Worker_Bee(tasks,results,i))

    for worker in workers:
        worker.start()  

    for i in range(1):
        tasks.put(Image_Processor(vs.read()))
        time.sleep(.5)

    tasks.join()
    while(True):
        result = results.get()
        print(result)
        if result[0]: #if detected
            cop.send_land_message_angular(result[1],result[2],result[3])
        time.sleep(.05)
    
    #tempory for testing
    #print("Exited loop, switching to land mode and de-arming, index: %d"%j)
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed = False
    
    #give up important resources
    so.teardown()
    

if __name__ == "__main__":
    main()
