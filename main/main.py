import sys
import socket
import multiprocessing
from motion_controller.motion_controller import MotionController
from input_controller.input_controller import InputController
from output_controller.output_controller import OutputController
from GUI_controller.GUI_controller import GUIController

sever_ip = '0.0.0.0'
sever_port = 8485


# Multiprocessing
def process_motion_controller(communication_queues):
    motion = MotionController(communication_queues)
    motion.do_process_events_from_queues()

def process_input_controller(communication_queues):
    InputController(communication_queues).run(communication_queues)

def process_output_controller(communication_queues):
    OutputController(communication_queues).run(communication_queues)

def process_GUI_controller(communication_queues):
    GUIController(communication_queues).GUI_comm()

# Queues
def create_controllers_queues():
    communication_queues = {'motion_queue': multiprocessing.Queue(1),
                            'socket_queue': multiprocessing.Queue(1)}

    return communication_queues

def close_controllers_queues(communication_queues):
    for queue in communication_queues.items():
        queue.close()
        queue.join_thread()

def main():
    communication_queues = create_controllers_queues()

    motion_controller = multiprocessing.Process(target=process_motion_controller, args=(communication_queues,))
    motion_controller.daemon = True

    input_controller = multiprocessing.Process(target=process_input_controller, args=(communication_queues,))
    input_controller.daemon = True

    output_controller = multiprocessing.Process(target=process_output_controller, args=(communication_queues,))
    output_controller.daemon = True

    GUI_controller = multiprocessing.Process(target=process_GUI_controller, args=(communication_queues,))
    GUI_controller.daemon = True

    # Start the processes
    motion_controller.start()
    input_controller.start()
    output_controller.start()
    GUI_controller.start()

    # Make sure all controllers are working
    if not motion_controller.is_alive():
        print("Motion Controller is not working")
        sys.exit(1)
    if not input_controller.is_alive():
        print("Input Controller is not working")
        sys.exit(1)
    if not output_controller.is_alive():
        print("Output Controller is not working")
        sys.exit(1)
    if not GUI_controller.is_alive():
        print("GUI Controller is not working")
        sys.exit(1)

    # Wait till the processes end
    motion_controller.join()
    input_controller.join()
    output_controller.join()
    GUI_controller.join()

    close_controllers_queues(communication_queues)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Terminated due Control+C was pressed')
    else:
        print('Normal termination')
