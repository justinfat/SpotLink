#Latest version 20210909
import signal
import sys
import math
import queue
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from utilities.log import Logger
from utilities.config import Config

log = Logger().setup_logger('Motion controller')

# Body size
leg_len = 10.8 # leg length in cm
feet_len = 11.6 # feet length in cm
shoulder_len = 5.84 # shoulder length in cm

# For walk motion script
walk_mode = -1
Py_ground_front = -15
Py_ground_rear = -15
y_lift = 4.5
y_right_corrected = -1
Px_init_front = 1
Px_init_rear = -1
x_step = 2.5
Px_walk_front = [0, 0, 0, 0, 0, 0, 0]
Px_walk_rear = [0, 0, 0, 0, 0, 0, 0]
for i in range(7):
    Px_walk_front[i] = Px_init_front + x_step*(3 - i)
    Px_walk_rear[i] = Px_init_rear + x_step*(3 - i)

# calibration adjustment
front_left_init = (86, 160, 6)
front_right_init = (90, 18, 170)
rear_left_init = (95, 165, 0)
rear_right_init = (92, 18, 166)
leg_init_angle = 15 # initial angle with horizontal
feet_init_angle = 45 # initial angle with leg

#position
rear_left_P = [0, 0, 0]
rear_right_P = [0, 0, 0]
front_left_P = [0, 0, 0]
front_right_P = [0, 0, 0]
rear_left_P0 = [0, 0, 0]
rear_right_P0 = [0, 0, 0]
front_left_P0 = [0, 0, 0]
front_right_P0 = [0, 0, 0]

class MotionController:
    ## declare variable ##
    boards = 1

    is_activated = False

    i2c = None
    pca9685_1 = None
    pca9685_2 = None

    pca9685_1_address = None
    pca9685_1_reference_clock_speed = None
    pca9685_1_frequency = None
    pca9685_2_address = None
    pca9685_2_reference_clock_speed = None
    pca9685_2_frequency = None

    servo_rear_shoulder_left = None
    servo_rear_shoulder_left_pca9685 = None
    servo_rear_shoulder_left_channel = None
    servo_rear_shoulder_left_min_pulse = None
    servo_rear_shoulder_left_max_pulse = None
    servo_rear_shoulder_left_rest_angle = None

    servo_rear_leg_left = None
    servo_rear_leg_left_pca9685 = None
    servo_rear_leg_left_channel = None
    servo_rear_leg_left_min_pulse = None
    servo_rear_leg_left_max_pulse = None
    servo_rear_leg_left_rest_angle = None

    servo_rear_feet_left = None
    servo_rear_feet_left_pca9685 = None
    servo_rear_feet_left_channel = None
    servo_rear_feet_left_min_pulse = None
    servo_rear_feet_left_max_pulse = None
    servo_rear_feet_left_rest_angle = None

    servo_rear_shoulder_right = None
    servo_rear_shoulder_right_pca9685 = None
    servo_rear_shoulder_right_channel = None
    servo_rear_shoulder_right_min_pulse = None
    servo_rear_shoulder_right_max_pulse = None
    servo_rear_shoulder_right_rest_angle = None

    servo_rear_leg_right = None
    servo_rear_leg_right_pca9685 = None
    servo_rear_leg_right_channel = None
    servo_rear_leg_right_min_pulse = None
    servo_rear_leg_right_max_pulse = None
    servo_rear_leg_right_rest_angle = None

    servo_rear_feet_right = None
    servo_rear_feet_right_pca9685 = None
    servo_rear_feet_right_channel = None
    servo_rear_feet_right_min_pulse = None
    servo_rear_feet_right_max_pulse = None
    servo_rear_feet_right_rest_angle = None

    servo_front_shoulder_left = None
    servo_front_shoulder_left_pca9685 = None
    servo_front_shoulder_left_channel = None
    servo_front_shoulder_left_min_pulse = None
    servo_front_shoulder_left_max_pulse = None
    servo_front_shoulder_left_rest_angle = None

    servo_front_leg_left = None
    servo_front_leg_left_pca9685 = None
    servo_front_leg_left_channel = None
    servo_front_leg_left_min_pulse = None
    servo_front_leg_left_max_pulse = None
    servo_front_leg_left_rest_angle = None

    servo_front_feet_left = None
    servo_front_feet_left_pca9685 = None
    servo_front_feet_left_channel = None
    servo_front_feet_left_min_pulse = None
    servo_front_feet_left_max_pulse = None
    servo_front_feet_left_rest_angle = None

    servo_front_shoulder_right = None
    servo_front_shoulder_right_pca9685 = None
    servo_front_shoulder_right_channel = None
    servo_front_shoulder_right_min_pulse = None
    servo_front_shoulder_right_max_pulse = None
    servo_front_shoulder_right_rest_angle = None

    servo_front_leg_right = None
    servo_front_leg_right_pca9685 = None
    servo_front_leg_right_channel = None
    servo_front_leg_right_min_pulse = None
    servo_front_leg_right_max_pulse = None
    servo_front_leg_right_rest_angle = None

    servo_front_feet_right = None
    servo_front_feet_right_pca9685 = None
    servo_front_feet_right_channel = None
    servo_front_feet_right_min_pulse = None
    servo_front_feet_right_max_pulse = None
    servo_front_feet_right_rest_angle = None

    # servo_arm_rotation = None
    # servo_arm_rotation_pca9685 = None
    # servo_arm_rotation_channel = None
    # servo_arm_rotation_min_pulse = None
    # servo_arm_rotation_max_pulse = None
    # servo_arm_rotation_rest_angle = None

    # servo_arm_lift = None
    # servo_arm_lift_pca9685 = None
    # servo_arm_lift_channel = None
    # servo_arm_lift_min_pulse = None
    # servo_arm_lift_max_pulse = None
    # servo_arm_lift_rest_angle = None

    # servo_arm_range = None
    # servo_arm_range_pca9685 = None
    # servo_arm_range_channel = None
    # servo_arm_range_min_pulse = None
    # servo_arm_range_max_pulse = None
    # servo_arm_range_rest_angle = None

    # servo_arm_cam_tilt = None
    # servo_arm_cam_tilt_pca9685 = None
    # servo_arm_cam_tilt_channel = None
    # servo_arm_cam_tilt_min_pulse = None
    # servo_arm_cam_tilt_max_pulse = None
    # servo_arm_cam_tilt_rest_angle = None

    def __init__(self, communication_queues): # run when class MotionController is created

        try:

            # log.debug('Starting controller...')
            print('initializing motion controller...')

            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            self.i2c = busio.I2C(SCL, SDA)
            self.load_pca9685_boards_configuration()
            self.load_servos_configuration()
            self.activate_pca9685_boards()
            self.activate_servos()

            self._abort_queue = communication_queues['abort_controller']
            self._motion_queue = communication_queues['motion_controller']

            self._previous_event = {}

            # self.init_position()
            

        except Exception as e:
            log.error('Motion controller initialization problem', e)
            try:
                self.pca9685_1.deinit()
            finally:
                try:
                    if self.boards == 2:
                        self.pca9685_2.deinit()
                finally:
                    sys.exit(1)

    def exit_gracefully(self, signum, frame):
        try:
            self.pca9685_1.deinit()
        finally:
            try:
                if self.boards == 2:
                    self.pca9685_2.deinit()
            finally:
                log.info('Terminated')
                sys.exit(0)

    def do_process_events_from_queues(self):

        while True:
            try:
                event = self._motion_queue.get(block=True, timeout=60)

                # send_controller
                if event == 'TooRight':
                    print('Too right...')
                if event == 'TooLeft':
                    self.init_position()
                    print('Too left...')
                if event == 'TooLow':
                    print('Too low...')
                if event == 'TooHigh':
                    print('Too high...')
                
                # #serial controller
                # if type(event) == int:
                #     if event == 11519:
                #         self.walk_stably()
                #     if event == 21074: #right
                #         self.shift_z(1)
                #         #self.rotate_CW()
                #         #self._serial_queue.put(0)
                #     if event == 19532: #left
                #         self.shift_z(-1)
                #         #self.rotate_CCW()
                #         #self._serial_queue.put(0)
                # else:
                #     #gamepad controller
                #     if event['start']:
                #         if self.is_activated:
                #             self.init_position()
                #             time.sleep(0.5)
                #             self.deactivate_pca9685_boards()
                #             self._abort_queue.put('abort')
                #         else:
                #             self._abort_queue.put('activate')
                #             self.activate_pca9685_boards()
                #             self.activate_servos()
                #             self.init_position()

                #     if not self.is_activated:
                #         log.info('Press START/OPTIONS to enable the servos')
                #         continue
                    
                #     if event['mode']: #PS button
                #         self.init_position()
                #         log.info('resting')

                #     #if event['a']: # buttom
                #     #    self.stand()
                        

                #     if event['x']: #top
                #         self.walk_stably()

                #     if event['y']: #left
                #         self.start_walk()

                #     #if event['b']: #right
                #     #    self.stop_walk()
                    
                #     ## body posture ##
                #     if event['select']: #share
                #         self.stand()
                #         log.info('standing...')
                #         log.info(self.servo_rear_shoulder_left.angle, self.servo_rear_shoulder_right.angle, self.servo_front_shoulder_left.angle, self.servo_front_shoulder_right.angle)

                #     if event['hat0x']: #direction button x
                #         self.shift_z(event['hat0x'])

                #     if event['hat0y']: #direction button y
                #         self.body_move_y(event['hat0y'])

                #     if event['ry']: #right joystick y
                #         self.body_move_analog_x(event['ry'])

                #     if event['rx']: #right joystick x
                #         self.body_move_analog_z(event['rx'])

                #     if event['tr']: #R1
                #         self.rotate_CW()

                #     if event['tl']: #L1
                #         self.rotate_CCW()
                    
                    # if event['hat0x'] and event['tl2']:
                    #     # 2 buttons example
                    #     pass

                    #self.move()

            except queue.Empty as e:
                log.info('Inactivity lasted 60 seconds, shutting down the servos, '
                         'press start to reactivate')
                if self.is_activated:
                    self.init_position()
                    time.sleep(0.5)
                    self.deactivate_pca9685_boards()
            
            except Exception as e:
                print(e)
                # log.error('Unknown problem while processing the queue of the motion controller')
                # log.error(' - Most likely a servo is not able to get to the assigned position')
            
    ##### SETUP ##### 
    def load_pca9685_boards_configuration(self):
        self.pca9685_1_address = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_ADDRESS), 0)
        self.pca9685_1_reference_clock_speed = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_REFERENCE_CLOCK_SPEED))
        self.pca9685_1_frequency = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_FREQUENCY))

        self.pca9685_2_address = False
        try:
            self.pca9685_2_address = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_ADDRESS), 0)

            if self.pca9685_2_address:
                self.pca9685_2_reference_clock_speed = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_REFERENCE_CLOCK_SPEED))
                self.pca9685_2_frequency = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_FREQUENCY))

        except Exception as e:
            log.debug("Only 1 PCA9685 is present in the configuration")
    def activate_pca9685_boards(self):

        self.pca9685_1 = PCA9685(self.i2c, address=self.pca9685_1_address,
                                 reference_clock_speed=self.pca9685_1_reference_clock_speed)
        self.pca9685_1.frequency = self.pca9685_1_frequency

        if self.pca9685_2_address:
            self.pca9685_2 = PCA9685(self.i2c, address=self.pca9685_2_address,
                                     reference_clock_speed=self.pca9685_2_reference_clock_speed)
            self.pca9685_2.frequency = self.pca9685_2_frequency
            self.boards = 2

        self.is_activated = True
        log.debug(str(self.boards) + ' PCA9685 board(s) activated')
    def deactivate_pca9685_boards(self):

        try:
            if self.pca9685_1:
                self.pca9685_1.deinit()
        finally:
            try:
                if self.boards == 2 and self.pca9685_2:
                    self.pca9685_2.deinit()
            finally:
                # self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                self.is_activated = False

        log.debug(str(self.boards) + ' PCA9685 board(s) deactivated')
    def load_servos_configuration(self):

        self.servo_rear_shoulder_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)
        self.servo_rear_shoulder_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL)
        self.servo_rear_shoulder_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MIN_PULSE)
        self.servo_rear_shoulder_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MAX_PULSE)
        self.servo_rear_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)

        self.servo_rear_leg_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)
        self.servo_rear_leg_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL)
        self.servo_rear_leg_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MIN_PULSE)
        self.servo_rear_leg_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MAX_PULSE)
        self.servo_rear_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)

        self.servo_rear_feet_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685)
        self.servo_rear_feet_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL)
        self.servo_rear_feet_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MIN_PULSE)
        self.servo_rear_feet_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MAX_PULSE)
        self.servo_rear_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)

        self.servo_rear_shoulder_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)
        self.servo_rear_shoulder_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL)
        self.servo_rear_shoulder_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MIN_PULSE)
        self.servo_rear_shoulder_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MAX_PULSE)
        self.servo_rear_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)

        self.servo_rear_leg_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)
        self.servo_rear_leg_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL)
        self.servo_rear_leg_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MIN_PULSE)
        self.servo_rear_leg_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MAX_PULSE)
        self.servo_rear_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)

        self.servo_rear_feet_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685)
        self.servo_rear_feet_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL)
        self.servo_rear_feet_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MIN_PULSE)
        self.servo_rear_feet_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MAX_PULSE)
        self.servo_rear_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)

        self.servo_front_shoulder_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)
        self.servo_front_shoulder_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL)
        self.servo_front_shoulder_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MIN_PULSE)
        self.servo_front_shoulder_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MAX_PULSE)
        self.servo_front_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)

        self.servo_front_leg_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)
        self.servo_front_leg_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL)
        self.servo_front_leg_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MIN_PULSE)
        self.servo_front_leg_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MAX_PULSE)
        self.servo_front_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)

        self.servo_front_feet_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685)
        self.servo_front_feet_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL)
        self.servo_front_feet_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MIN_PULSE)
        self.servo_front_feet_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MAX_PULSE)
        self.servo_front_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)

        self.servo_front_shoulder_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)
        self.servo_front_shoulder_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL)
        self.servo_front_shoulder_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MIN_PULSE)
        self.servo_front_shoulder_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MAX_PULSE)
        self.servo_front_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)

        self.servo_front_leg_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)
        self.servo_front_leg_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL)
        self.servo_front_leg_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MIN_PULSE)
        self.servo_front_leg_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MAX_PULSE)
        self.servo_front_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)

        self.servo_front_feet_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685)
        self.servo_front_feet_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL)
        self.servo_front_feet_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MIN_PULSE)
        self.servo_front_feet_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MAX_PULSE)
        self.servo_front_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)

        # if self.servo_arm_rotation_pca9685:
        #     self.servo_arm_rotation_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685)
        #     self.servo_arm_rotation_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL)
        #     self.servo_arm_rotation_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_MIN_PULSE)
        #     self.servo_arm_rotation_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_MAX_PULSE)
        #     self.servo_arm_rotation_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)

        #     self.servo_arm_lift_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685)
        #     self.servo_arm_lift_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL)
        #     self.servo_arm_lift_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_MIN_PULSE)
        #     self.servo_arm_lift_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_MAX_PULSE)
        #     self.servo_arm_lift_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)

        #     self.servo_arm_range_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685)
        #     self.servo_arm_range_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL)
        #     self.servo_arm_range_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_MIN_PULSE)
        #     self.servo_arm_range_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_MAX_PULSE)
        #     self.servo_arm_range_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)

        #     self.servo_arm_cam_tilt_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685)
        #     self.servo_arm_cam_tilt_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL)
        #     self.servo_arm_cam_tilt_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_MIN_PULSE)
        #     self.servo_arm_cam_tilt_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_MAX_PULSE)
        #     self.servo_arm_cam_tilt_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)
    def activate_servos(self):

        if self.servo_rear_shoulder_left_pca9685 == 1:
            self.servo_rear_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_left_channel])
        else:
            self.servo_rear_shoulder_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_shoulder_left_channel])
        self.servo_rear_shoulder_left.set_pulse_width_range(min_pulse=self.servo_rear_shoulder_left_min_pulse, max_pulse=self.servo_rear_shoulder_left_max_pulse)

        if self.servo_rear_leg_left_pca9685 == 1:
            self.servo_rear_leg_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_leg_left_channel])
        else:
            self.servo_rear_leg_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_leg_left_channel])
        self.servo_rear_leg_left.set_pulse_width_range(min_pulse=self.servo_rear_leg_left_min_pulse, max_pulse=self.servo_rear_leg_left_max_pulse)

        if self.servo_rear_feet_left_pca9685 == 1:
            self.servo_rear_feet_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_feet_left_channel])
        else:
            self.servo_rear_feet_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_feet_left_channel])
        self.servo_rear_feet_left.set_pulse_width_range(min_pulse=self.servo_rear_feet_left_min_pulse, max_pulse=self.servo_rear_feet_left_max_pulse)

        if self.servo_rear_shoulder_right_pca9685 == 1:
            self.servo_rear_shoulder_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_right_channel])
        else:
            self.servo_rear_shoulder_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_shoulder_right_channel])
        self.servo_rear_shoulder_right.set_pulse_width_range(min_pulse=self.servo_rear_shoulder_right_min_pulse, max_pulse=self.servo_rear_shoulder_right_max_pulse)

        if self.servo_rear_leg_right_pca9685 == 1:
            self.servo_rear_leg_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_leg_right_channel])
        else:
            self.servo_rear_leg_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_leg_right_channel])
        self.servo_rear_leg_right.set_pulse_width_range(min_pulse=self.servo_rear_leg_right_min_pulse, max_pulse=self.servo_rear_leg_right_max_pulse)

        if self.servo_rear_feet_right_pca9685 == 1:
            self.servo_rear_feet_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_feet_right_channel])
        else:
            self.servo_rear_feet_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_feet_right_channel])
        self.servo_rear_feet_right.set_pulse_width_range(min_pulse=self.servo_rear_feet_right_min_pulse, max_pulse=self.servo_rear_feet_right_max_pulse)

        if self.servo_front_shoulder_left_pca9685 == 1:
            self.servo_front_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_front_shoulder_left_channel])
        else:
            self.servo_front_shoulder_left = servo.Servo(self.pca9685_2.channels[self.servo_front_shoulder_left_channel])
        self.servo_front_shoulder_left.set_pulse_width_range(min_pulse=self.servo_front_shoulder_left_min_pulse, max_pulse=self.servo_front_shoulder_left_max_pulse)

        if self.servo_front_leg_left_pca9685 == 1:
            self.servo_front_leg_left = servo.Servo(self.pca9685_1.channels[self.servo_front_leg_left_channel])
        else:
            self.servo_front_leg_left = servo.Servo(self.pca9685_2.channels[self.servo_front_leg_left_channel])
        self.servo_front_leg_left.set_pulse_width_range(min_pulse=self.servo_front_leg_left_min_pulse, max_pulse=self.servo_front_leg_left_max_pulse)

        if self.servo_front_feet_left_pca9685 == 1:
            self.servo_front_feet_left = servo.Servo(self.pca9685_1.channels[self.servo_front_feet_left_channel])
        else:
            self.servo_front_feet_left = servo.Servo(self.pca9685_2.channels[self.servo_front_feet_left_channel])
        self.servo_front_feet_left.set_pulse_width_range(min_pulse=self.servo_front_feet_left_min_pulse, max_pulse=self.servo_front_feet_left_max_pulse)

        if self.servo_front_shoulder_right_pca9685 == 1:
            self.servo_front_shoulder_right = servo.Servo(self.pca9685_1.channels[self.servo_front_shoulder_right_channel])
        else:
            self.servo_front_shoulder_right = servo.Servo(
                self.pca9685_2.channels[self.servo_front_shoulder_right_channel])
        self.servo_front_shoulder_right.set_pulse_width_range(min_pulse=self.servo_front_shoulder_right_min_pulse, max_pulse=self.servo_front_shoulder_right_max_pulse)

        if self.servo_front_leg_right_pca9685 == 1:
            self.servo_front_leg_right = servo.Servo(self.pca9685_1.channels[self.servo_front_leg_right_channel])
        else:
            self.servo_front_leg_right = servo.Servo(
                self.pca9685_2.channels[self.servo_front_leg_right_channel])
        self.servo_front_leg_right.set_pulse_width_range(min_pulse=self.servo_front_leg_right_min_pulse, max_pulse=self.servo_front_leg_right_max_pulse)

        if self.servo_front_feet_right_pca9685 == 1:
            self.servo_front_feet_right = servo.Servo(self.pca9685_1.channels[self.servo_front_feet_right_channel])
        else:
            self.servo_front_feet_right = servo.Servo(self.pca9685_2.channels[self.servo_front_feet_right_channel])
        self.servo_front_feet_right.set_pulse_width_range(min_pulse=self.servo_front_feet_right_min_pulse, max_pulse=self.servo_front_feet_right_max_pulse)

        # if self.servo_arm_rotation_pca9685:

        #     if self.servo_arm_rotation_pca9685 == 1:
        #         self.servo_arm_rotation = servo.Servo(self.pca9685_1.channels[self.servo_arm_rotation_channel])
        #     else:
        #         self.servo_arm_rotation = servo.Servo(self.pca9685_2.channels[self.servo_arm_rotation_channel])
        #     self.servo_arm_rotation.set_pulse_width_range(min_pulse=self.servo_arm_rotation_min_pulse, max_pulse=self.servo_arm_rotation_max_pulse)

        #     if self.servo_arm_lift_pca9685 == 1:
        #         self.servo_arm_lift = servo.Servo(self.pca9685_1.channels[self.servo_arm_lift_channel])
        #     else:
        #         self.servo_arm_lift = servo.Servo(self.pca9685_2.channels[self.servo_arm_lift_channel])
        #     self.servo_arm_lift.set_pulse_width_range(min_pulse=self.servo_arm_lift_min_pulse, max_pulse=self.servo_arm_lift_max_pulse)

        #     if self.servo_arm_range_pca9685 == 1:
        #         self.servo_arm_range = servo.Servo(self.pca9685_1.channels[self.servo_arm_range_channel])
        #    else:
        #        self.servo_arm_range = servo.Servo(self.pca9685_2.channels[self.servo_arm_range_channel])
        #    self.servo_arm_range.set_pulse_width_range(min_pulse=self.servo_arm_range_min_pulse, max_pulse=self.servo_arm_range_max_pulse)

        #   if self.servo_arm_cam_tilt_pca9685 == 1:
        #        self.servo_arm_cam_tilt = servo.Servo(self.pca9685_1.channels[self.servo_arm_cam_tilt_channel])
        #    else:
        #        self.servo_arm_cam_tilt = servo.Servo(self.pca9685_2.channels[self.servo_arm_cam_tilt_channel])
        #    self.servo_arm_cam_tilt.set_pulse_width_range(min_pulse=self.servo_arm_cam_tilt_min_pulse, max_pulse=self.servo_arm_cam_tilt_max_pulse)
    
    ##### MOVE FUNCTIONS #####
    ## P position: new position to move ##
    def rear_left_position_P(self, Px, Py, Pz):
        rear_left_P[0] = Px
        rear_left_P[1] = Py
        rear_left_P[2] = Pz
    def rear_right_position_P(self, Px, Py, Pz):
        rear_right_P[0] = Px
        rear_right_P[1] = Py
        rear_right_P[2] = Pz
    def front_left_position_P(self, Px, Py, Pz):
        front_left_P[0] = Px
        front_left_P[1] = Py
        front_left_P[2] = Pz
    def front_right_position_P(self, Px, Py, Pz):
        front_right_P[0] = Px
        front_right_P[1] = Py
        front_right_P[2] = Pz
    ## P0 position: last position ##
    def rear_left_position_P0(self, Px, Py, Pz):
        rear_left_P0[0] = Px
        rear_left_P0[1] = Py
        rear_left_P0[2] = Pz
    def rear_right_position_P0(self, Px, Py, Pz):
        rear_right_P0[0] = Px
        rear_right_P0[1] = Py
        rear_right_P0[2] = Pz
    def front_left_position_P0(self, Px, Py, Pz):
        front_left_P0[0] = Px
        front_left_P0[1] = Py
        front_left_P0[2] = Pz
    def front_right_position_P0(self, Px, Py, Pz):
        front_right_P0[0] = Px
        front_right_P0[1] = Py
        front_right_P0[2] = Pz
    
    ## get motor angle ##
    def getPhi(self, Px, Py, Pz): 
        Pz_corrected = Pz + shoulder_len 
        h = math.sqrt(Py**2 + Pz_corrected**2 - shoulder_len**2) # length of leg plus feet
        phi = math.acos((Px**2 + h**2 - leg_len**2 - feet_len**2)/(-2*leg_len*feet_len))*180/math.pi
        return phi
    def getTheta(self, Px, Py, Pz):
        Pz_corrected = Pz + shoulder_len 
        h = math.sqrt(Py**2 + Pz_corrected**2 - shoulder_len**2)
        gamma = math.acos(Px/math.sqrt(Px**2 + h**2))*180/math.pi
        beta = math.asin(feet_len * math.sin((self.getPhi(Px, Py, Pz))*math.pi/180)/math.sqrt(Px**2 + h**2))*180/math.pi
        theta = 180 - beta - gamma
        return theta      
    def getDelta(self, Px, Py, Pz):
        Pz_corrected = Pz + shoulder_len 
        h = math.sqrt(Py**2 + Pz_corrected**2 - shoulder_len**2)
        delta = math.asin((h*Pz_corrected + shoulder_len*Py)/(h**2 + shoulder_len**2))*180/math.pi
        return delta
    ## make each leg move ## 
    def rear_left_move(self, Px, Py, Pz):
        shoulder_angle = rear_left_init[0] + round(self.getDelta(Px, Py, Pz))
        leg_angle = rear_left_init[1] - (round(self.getTheta(Px, Py, Pz))-leg_init_angle)
        feet_angle = rear_left_init[2] + (round(self.getPhi(Px, Py, Pz))-feet_init_angle)

        self.servo_rear_shoulder_left.angle = shoulder_angle
        self.servo_rear_leg_left.angle = leg_angle
        self.servo_rear_feet_left.angle = feet_angle
    def rear_right_move(self, Px, Py, Pz):
        shoulder_angle = rear_right_init[0] - round(self.getDelta(Px, Py, Pz))
        leg_angle = rear_right_init[1] + (round(self.getTheta(Px, Py, Pz))-leg_init_angle)
        feet_angle = rear_right_init[2] - (round(self.getPhi(Px, Py, Pz))-feet_init_angle)

        self.servo_rear_shoulder_right.angle = shoulder_angle
        self.servo_rear_leg_right.angle = leg_angle
        self.servo_rear_feet_right.angle = feet_angle
    def front_left_move(self, Px, Py, Pz):
        shoulder_angle = front_left_init[0] - round(self.getDelta(Px, Py, Pz))
        leg_angle = front_left_init[1] - (round(self.getTheta(Px, Py, Pz))-leg_init_angle)
        feet_angle = front_left_init[2] + (round(self.getPhi(Px, Py, Pz))-feet_init_angle)

        self.servo_front_shoulder_left.angle = shoulder_angle
        self.servo_front_leg_left.angle = leg_angle
        self.servo_front_feet_left.angle = feet_angle
    def front_right_move(self, Px, Py, Pz):
        shoulder_angle = front_right_init[0] + round(self.getDelta(Px, Py, Pz))
        leg_angle = front_right_init[1] + (round(self.getTheta(Px, Py, Pz))-leg_init_angle)
        feet_angle = front_right_init[2] - (round(self.getPhi(Px, Py, Pz))-feet_init_angle)

        self.servo_front_shoulder_right.angle = shoulder_angle
        self.servo_front_leg_right.angle = leg_angle
        self.servo_front_feet_right.angle = feet_angle
    ## make all legs move together ##
    def move_all(self,interval_num):
        if interval_num == 0:
            self.rear_left_move(rear_left_P[0], rear_left_P[1], rear_left_P[2])
            self.rear_right_move(rear_right_P[0], rear_right_P[1], rear_right_P[2])
            self.front_left_move(front_left_P[0], front_left_P[1], front_left_P[2])
            self.front_right_move(front_right_P[0], front_right_P[1], front_right_P[2])
        else:
            rear_left_interval = [0, 0, 0]
            rear_right_interval = [0, 0, 0]
            front_left_interval = [0, 0, 0]
            front_right_interval = [0, 0, 0]
            for i in [0, 1, 2]:
                rear_left_interval[i] = (rear_left_P[i] - rear_left_P0[i])/interval_num
                rear_right_interval[i] = (rear_right_P[i] - rear_right_P0[i])/interval_num
                front_left_interval[i] = (front_left_P[i] - front_left_P0[i])/interval_num
                front_right_interval[i] = (front_right_P[i] - front_right_P0[i])/interval_num

            for i in range(1,interval_num+1):
                self.rear_left_move(rear_left_P0[0]+rear_left_interval[0]*i, rear_left_P0[1]+rear_left_interval[1]*i, rear_left_P[2]+rear_left_interval[2]*i)
                self.rear_right_move(rear_right_P0[0]+rear_right_interval[0]*i, rear_right_P0[1]+rear_right_interval[1]*i, rear_right_P[2]+rear_right_interval[2]*i)
                self.front_left_move(front_left_P0[0]+front_left_interval[0]*i, front_left_P0[1]+front_left_interval[1]*i, front_left_P[2]+front_left_interval[2]*i)
                self.front_right_move(front_right_P0[0]+front_right_interval[0]*i, front_right_P0[1]+front_right_interval[1]*i, front_right_P[2]+front_right_interval[2]*i)
                time.sleep(0.01)

        self.front_left_position_P0(front_left_P[0], front_left_P[1], front_left_P[2])
        self.front_right_position_P0(front_right_P[0], front_right_P[1], front_right_P[2])
        self.rear_left_position_P0(rear_left_P[0], rear_left_P[1], rear_left_P[2])
        self.rear_right_position_P0(rear_right_P[0], rear_right_P[1], rear_right_P[2])
        
    ##### MOTION SCRIPTS #####
    ## walk ##
    def start_walk(self):
        #shoulders
        #self.servo_rear_shoulder_left.angle = rear_left_init[0]
        #self.servo_rear_shoulder_right.angle = rear_right_init[0]
        #self.servo_front_shoulder_left.angle = front_left_init[0]
        #self.servo_front_shoulder_right.angle = front_right_init[0]

        if walk_mode == 1:
            #init_position
            self.front_left_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.move_all(10)
            #front_left_lift
            self.front_left_position_P(Px_walk_front[2], Py_ground_front + y_lift, 0)
            self.move_all(5)
            #front_right_lift
            self.front_left_position_P(Px_walk_front[0], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[3], Py_ground_front + y_lift, 0)
            self.move_all(5)
            #rear_left_lift
            self.front_right_position_P(Px_walk_front[4], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[3], Py_ground_rear + y_lift, 0)
            self.move_all(5)
            #rear_right_lift
            self.rear_left_position_P(Px_walk_rear[2], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[4], Py_ground_rear + y_lift, 0)
            self.move_all(5)
            #end_position
            self.rear_right_position_P(Px_walk_rear[6], Py_ground_rear, 0)
            self.move_all(5)
        elif walk_mode == -1:
            #init_position
            self.front_left_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.move_all(10)
            #front_right
            self.front_right_position_P(Px_walk_front[4], Py_ground_front + y_right_corrected + y_lift, 0)
            self.move_all(5)
            self.front_right_position_P(Px_walk_front[6], Py_ground_front + y_right_corrected, 0)
            self.move_all(5)
            #rear_right
            self.rear_right_position_P(Px_walk_rear[4], Py_ground_rear + y_right_corrected + y_lift, 0)
            self.move_all(5)
            self.rear_right_position_P(Px_walk_rear[6], Py_ground_rear + y_right_corrected, 0)
            self.move_all(5)
    def walk_stably(self):
        i = 5
        #shoulders
        #self.servo_rear_shoulder_left.angle = rear_left_init[0] 
        #self.servo_rear_shoulder_right.angle = rear_right_init[0]
        #self.servo_front_shoulder_left.angle = front_left_init[0]
        #self.servo_front_shoulder_right.angle = front_right_init[0]

        if walk_mode == 1:
            #rear_right_lift_1	
            self.front_left_position_P(Px_walk_front[1], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[5], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[4], Py_ground_rear + y_lift, 0)
            self.move_all(i)
            #rear_right_lift_2
            self.front_left_position_P(Px_walk_front[2], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[6], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[4], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[2], Py_ground_rear + y_lift, 0)
            self.move_all(i)
            #rear_right_ground
            self.rear_right_position_P(Px_walk_rear[0], Py_ground_rear, 0)
            self.move_all(i)

            #front_right_lift_1
            self.front_left_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[4], Py_ground_front + y_lift, 0)
            self.rear_left_position_P(Px_walk_rear[5], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[0], Py_ground_rear, 0)
            self.move_all(i)
            #front_right_lift_2
            self.front_left_position_P(Px_walk_front[4], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[2], Py_ground_front + y_lift, 0)
            self.rear_left_position_P(Px_walk_rear[6], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[2], Py_ground_rear, 0)
            self.move_all(i)
            #front_right_ground
            self.front_right_position_P(Px_walk_front[0], Py_ground_front, 0)
            self.move_all(i)

            #rear_left_lift_1
            self.front_left_position_P(Px_walk_front[5], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[1], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[4], Py_ground_rear + y_lift, 0)
            self.rear_right_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.move_all(i)
            #rear_left_lift_2
            self.front_left_position_P(Px_walk_front[6], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[2], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[2], Py_ground_rear + y_lift, 0)
            self.rear_right_position_P(Px_walk_rear[4], Py_ground_rear, 0)
            self.move_all(i)
            #rear_left_ground
            self.rear_left_position_P(Px_walk_rear[0], Py_ground_rear, 0)
            self.move_all(i)

            #front_left_lift_1
            self.front_left_position_P(Px_walk_front[4], Py_ground_front + y_lift, 0)
            self.front_right_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[1], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[5], Py_ground_rear, 0)
            self.move_all(i)
            #front_left_lift_2
            self.front_left_position_P(Px_walk_front[2], Py_ground_front + y_lift, 0)
            self.front_right_position_P(Px_walk_front[4], Py_ground_front, 0)
            self.rear_left_position_P(Px_walk_rear[2], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[6], Py_ground_rear, 0)
            self.move_all(i)
            #front_left_ground
            self.front_left_position_P(Px_walk_front[0], Py_ground_front, 0)
            self.move_all(i)
            #stand_position
            #self.front_left_position_P(Px_walk_front[0], Py_ground_front, 0)
            #self.front_right_position_P(Px_walk_front[4], Py_ground_front, 0)
            #self.rear_left_position_P(Px_walk_rear[2], Py_ground_rear, 0)
            #self.rear_right_position_P(Px_walk_rear[6], Py_ground_rear, 0)
            #self.move_all(i)
        elif walk_mode == -1:
            #rear_right_lift
            self.rear_right_position_P(Px_walk_rear[3], Py_ground_rear + y_right_corrected + y_lift, 0)
            self.move_all(i)
            #rear_right_ground
            self.rear_right_position_P(Px_walk_rear[0], Py_ground_rear + y_right_corrected +1, 0)
            self.move_all(i)
            #front_right_lift
            self.front_right_position_P(Px_walk_front[3], Py_ground_front + y_right_corrected + y_lift, 0)
            self.move_all(i)
            #front_right_ground
            self.front_left_position_P(Px_walk_front[6], Py_ground_front -1, 0)
            self.front_right_position_P(Px_walk_front[3], Py_ground_front + y_right_corrected, 0)
            self.rear_left_position_P(Px_walk_rear[6], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[3], Py_ground_rear + y_right_corrected, 0)
            self.move_all(i*3)
            #rear_left_lift
            self.rear_left_position_P(Px_walk_rear[3], Py_ground_rear + y_lift, 0)
            self.move_all(i)
            #rear_left_ground
            self.rear_left_position_P(Px_walk_rear[0], Py_ground_rear, 0)
            self.move_all(i)
            #front_left_lift
            self.front_left_position_P(Px_walk_front[3], Py_ground_front + y_lift, 0)
            self.move_all(i)
            #front_left_ground
            self.front_left_position_P(Px_walk_front[3], Py_ground_front, 0)
            self.front_right_position_P(Px_walk_front[6], Py_ground_front + y_right_corrected, 0)
            self.rear_left_position_P(Px_walk_rear[3], Py_ground_rear, 0)
            self.rear_right_position_P(Px_walk_rear[6], Py_ground_rear + y_right_corrected, 0)
            self.move_all(i*3)
    #def stop_walk(self):

    ## shift ##
    def shift_z(self, raw_value):
        dis = 3
        lift = 5
        spd = 5
        if raw_value < 0: #left
            #front left 
            self.front_left_position_P(front_left_P[0], front_left_P[1]+lift, front_left_P[2])
            self.move_all(spd)
            self.front_left_position_P(front_left_P[0], front_left_P[1]-lift, front_left_P[2]+dis)
            self.move_all(spd)
            #rear left
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1]+lift, rear_left_P[2])
            self.move_all(spd)
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1]-lift, rear_left_P[2]+dis)
            self.move_all(spd)
            #body
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1], rear_left_P[2]-dis)
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1], rear_right_P[2]+dis)
            self.front_left_position_P(front_left_P[0], front_left_P[1], front_left_P[2]-dis)
            self.front_right_position_P(front_right_P[0], front_right_P[1], front_right_P[2]+dis)
            self.move_all(spd)
            #front right
            self.front_right_position_P(front_right_P[0], front_right_P[1]+lift, front_right_P[2]-dis)
            self.move_all(spd)
            self.front_right_position_P(front_right_P[0], front_right_P[1]-lift, front_right_P[2])
            self.move_all(spd)
            #rear right
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1]+lift, rear_right_P[2]-dis)
            self.move_all(spd)
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1]-lift, rear_right_P[2])
            self.move_all(spd)
        elif raw_value > 0: #right
            #front right
            self.front_right_position_P(front_right_P[0], front_right_P[1]+lift, front_right_P[2])
            self.move_all(spd)
            self.front_right_position_P(front_right_P[0], front_right_P[1]-lift, front_right_P[2]+dis)
            self.move_all(spd)
            #rear right
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1]+lift, rear_right_P[2])
            self.move_all(spd)
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1]-lift, rear_right_P[2]+dis)
            self.move_all(spd)
            #body
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1], rear_left_P[2]+dis)
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1], rear_right_P[2]-dis)
            self.front_left_position_P(front_left_P[0], front_left_P[1], front_left_P[2]+dis)
            self.front_right_position_P(front_right_P[0], front_right_P[1], front_right_P[2]-dis)
            self.move_all(spd)
            #front left
            self.front_left_position_P(front_left_P[0], front_left_P[1]+lift, front_left_P[2]-dis)
            self.move_all(spd)
            self.front_left_position_P(front_left_P[0], front_left_P[1]-lift, front_left_P[2])
            self.move_all(spd)
            #rear left
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1]+lift, rear_left_P[2]-dis)
            self.move_all(spd)
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1]-lift, rear_left_P[2])
            self.move_all(spd)

    ## rotate ##
    def rotate_CW(self):
        lift = 5
        spd = 5
        dis = 3

        #front right
        self.front_right_position_P(front_right_P[0], front_right_P[1]+lift, front_right_P[2])
        self.move_all(spd)
        self.front_right_position_P(front_right_P[0]-dis, front_right_P[1]-lift, front_right_P[2]+dis)
        self.move_all(spd)
        #rear right 
        self.rear_right_position_P(rear_right_P[0], rear_right_P[1]+lift, rear_right_P[2])
        self.move_all(spd)
        self.rear_right_position_P(rear_right_P[0]-dis, rear_right_P[1]-lift, rear_right_P[2]-dis)
        self.move_all(spd)
        #front left 
        self.front_left_position_P(front_left_P[0], front_left_P[1]+lift, front_left_P[2])
        self.move_all(spd)
        self.front_left_position_P(front_left_P[0]+dis, front_left_P[1]-lift, front_left_P[2]-dis)
        self.move_all(spd)
        #rear left 
        self.rear_left_position_P(rear_left_P[0], rear_left_P[1]+lift, rear_left_P[2])
        self.move_all(spd)
        self.rear_left_position_P(rear_left_P[0]+dis, rear_left_P[1]-lift, rear_left_P[2]+dis)
        self.move_all(spd)
        #body
        self.front_right_position_P(front_right_P[0]+dis, front_right_P[1], front_right_P[2]-dis)
        self.rear_right_position_P(rear_right_P[0]+dis, rear_right_P[1], rear_right_P[2]+dis)
        self.front_left_position_P(front_left_P[0]-dis, front_left_P[1], front_left_P[2]+dis)
        self.rear_left_position_P(rear_left_P[0]-dis, rear_left_P[1], rear_left_P[2]-dis)
        self.move_all(spd)
        #WHY???????????
        self.stand()
    def rotate_CCW(self):
        lift = 5
        spd = 5
        dis = 3

        #front left
        self.front_left_position_P(front_left_P[0], front_left_P[1]+lift, front_left_P[2])
        self.move_all(spd)
        self.front_left_position_P(front_left_P[0]-dis, front_left_P[1]-lift, front_left_P[2]+dis)
        self.move_all(spd)
        #rear left 
        self.rear_left_position_P(rear_left_P[0], rear_left_P[1]+lift, rear_left_P[2])
        self.move_all(spd)
        self.rear_left_position_P(rear_left_P[0]-dis, rear_left_P[1]-lift, rear_left_P[2]-dis)
        self.move_all(spd)
        #front right 
        self.front_right_position_P(front_right_P[0], front_right_P[1]+lift, front_right_P[2])
        self.move_all(spd)
        self.front_right_position_P(front_right_P[0]+dis, front_right_P[1]-lift, front_right_P[2]-dis)
        self.move_all(spd)
        #rear right 
        self.rear_right_position_P(rear_right_P[0], rear_right_P[1]+lift, rear_right_P[2])
        self.move_all(spd)
        self.rear_right_position_P(rear_right_P[0]+dis, rear_right_P[1]-lift, rear_right_P[2]+dis)
        self.move_all(spd)
        #body
        self.front_left_position_P(front_left_P[0]+dis, front_left_P[1], front_left_P[2]-dis)
        self.rear_left_position_P(rear_left_P[0]+dis, rear_left_P[1], rear_left_P[2]+dis)
        self.front_right_position_P(front_right_P[0]-dis, front_right_P[1], front_right_P[2]+dis)
        self.rear_right_position_P(rear_right_P[0]-dis, rear_right_P[1], rear_right_P[2]-dis)
        self.move_all(spd)
        #WHY???????????
        self.stand()

    ## body posture ##
    def stand(self):
        Px_front = 0
        Px_rear = 0
        Py = -10
        Pz = 0

        self.rear_left_position_P(Px_rear, Py, Pz)
        self.rear_right_position_P(Px_rear, Py, Pz)
        self.front_left_position_P(Px_front, Py, Pz)
        self.front_right_position_P(Px_front, Py, Pz)
        self.move_all(30)
    def body_move_y(self, raw_value):
        if raw_value < 0:
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1] - 1, rear_left_P[2])
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1] - 1, rear_right_P[2])
            self.front_left_position_P(front_left_P[0], front_left_P[1] - 1, front_left_P[2])
            self.front_right_position_P(front_right_P[0], front_right_P[1] - 1, front_right_P[2])
            self.move_all(5)
        elif raw_value > 0:
            self.rear_left_position_P(rear_left_P[0], rear_left_P[1] + 1, rear_left_P[2])
            self.rear_right_position_P(rear_right_P[0], rear_right_P[1] + 1, rear_right_P[2])
            self.front_left_position_P(front_left_P[0], front_left_P[1] + 1, front_left_P[2])
            self.front_right_position_P(front_right_P[0], front_right_P[1] + 1, front_right_P[2])
            self.move_all(5)
    def body_move_analog_x(self, raw_value):
        dis = self.maprange((-1, 1), (2, -2), raw_value)
        self.rear_left_position_P(-dis, rear_left_P[1], rear_left_P[2])
        self.rear_right_position_P(-dis, rear_right_P[1], rear_right_P[2])
        self.front_left_position_P(-dis, front_left_P[1], front_left_P[2])
        self.front_right_position_P(-dis, front_right_P[1], front_right_P[2])
        self.move_all(0)
    def body_move_analog_z(self, raw_value):
        dis = self.maprange((-1, 1), (-2.5, 2.5), raw_value)
        self.rear_left_position_P(rear_left_P[0], rear_left_P[1], dis)
        self.rear_right_position_P(rear_right_P[0], rear_right_P[1], -dis)
        self.front_left_position_P(front_left_P[0], front_left_P[1], dis)
        self.front_right_position_P(front_right_P[0], front_right_P[1], -dis)
        self.move_all(0)

    ## old functions (move by given angles) ##
    def move(self):

        try:
            self.servo_rear_shoulder_left.angle = self.servo_rear_shoulder_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_shoulder_left angle requested')

        try:
            self.servo_rear_leg_left.angle = self.servo_rear_leg_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_leg_left angle requested')

        try:
            self.servo_rear_feet_left.angle = self.servo_rear_feet_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_feet_left angle requested')

        try:
            self.servo_rear_shoulder_right.angle = self.servo_rear_shoulder_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_shoulder_right angle requested')

        try:
            self.servo_rear_leg_right.angle = self.servo_rear_leg_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_leg_right angle requested')

        try:
            self.servo_rear_feet_right.angle = self.servo_rear_feet_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_feet_right angle requested')

        try:
            self.servo_front_shoulder_left.angle = self.servo_front_shoulder_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_shoulder_left angle requested')

        try:
            self.servo_front_leg_left.angle = self.servo_front_leg_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_leg_left angle requested')

        try:
            self.servo_front_feet_left.angle = self.servo_front_feet_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_feet_left angle requested')

        try:
            self.servo_front_shoulder_right.angle = self.servo_front_shoulder_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_shoulder_right angle requested')

        try:
            self.servo_front_leg_right.angle = self.servo_front_leg_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_leg_right angle requested')

        try:
            self.servo_front_feet_right.angle = self.servo_front_feet_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_feet_right angle requested')

        # if self.servo_arm_rotation_pca9685:
        #     try:
        #         self.servo_arm_rotation.angle = self.servo_arm_rotation_rest_angle
        #     except ValueError as e:
        #         log.error('Impossible servo_arm_rotation angle requested')

        #     try:
        #         self.servo_arm_lift.angle = self.servo_arm_lift_rest_angle
        #     except ValueError as e:
        #         log.error('Impossible arm_lift angle requested')

        #     try:
        #         self.servo_arm_range.angle = self.servo_arm_range_rest_angle
        #     except ValueError as e:
        #         log.error('Impossible servo_arm_range angle requested')

        #     try:
        #         self.servo_arm_cam_tilt.angle = self.servo_arm_cam_tilt_rest_angle
        #     except ValueError as e:
        #         log.error('Impossible servo_arm_cam_tilt angle requested')
    def init_position(self):

        self.servo_rear_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)
        self.servo_rear_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)
        self.servo_rear_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)
        self.servo_rear_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)
        self.servo_rear_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)
        self.servo_rear_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)
        self.servo_front_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)
        self.servo_front_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)
        self.servo_front_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)
        self.servo_front_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)
        self.servo_front_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)
        self.servo_front_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)

        #shoulders
        self.servo_rear_shoulder_left.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)
        self.servo_rear_shoulder_right.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)
        self.servo_front_shoulder_left.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)
        self.servo_front_shoulder_right.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)

        #leg & feet
        self.servo_rear_leg_left.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)
        self.servo_rear_feet_left.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)
        self.servo_rear_leg_right.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)
        self.servo_rear_feet_right.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)
        self.servo_front_leg_left.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)
        self.servo_front_feet_left.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)
        self.servo_front_leg_right.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)
        self.servo_front_feet_right.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)

        init_Px = -leg_len*math.cos(math.pi*leg_init_angle/360) + feet_len*math.cos(math.pi*(feet_init_angle-leg_init_angle)/360)
        init_Py = -leg_len*math.sin(math.pi*leg_init_angle/36) - feet_len*math.sin(math.pi*(feet_init_angle-leg_init_angle)/360)
        
        self.rear_left_position_P(init_Px, init_Py, 0)
        self.rear_right_position_P(init_Px, init_Py, 0)
        self.front_left_position_P(init_Px, init_Py, 0)
        self.front_right_position_P(init_Px, init_Py, 0)

        self.rear_left_position_P0(init_Px, init_Py, 0)
        self.rear_right_position_P0(init_Px, init_Py, 0)
        self.front_left_position_P0(init_Px, init_Py, 0)
        self.front_right_position_P0(init_Px, init_Py, 0)

        # if self.servo_arm_rotation_pca9685:
        #     self.servo_arm_rotation.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)
        #     self.servo_arm_lift.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)
        #     self.servo_arm_range.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)
        #     self.servo_arm_cam_tilt.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)
  
    def maprange(self, a, b, s):
        (a1, a2), (b1, b2) = a, b
        return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))
