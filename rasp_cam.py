import os
from time import sleep
import RPi.GPIO as GPIO
import picamera


_img_dir = '/data/images/'
_img_prefix = 'sample_image_'
_video_dir = '/data/videos/'
_video_prefix = 'sample_video_'
_img_resolution = (640, 480)
_video_resolution = (640, 480)
_img_format = 'jpeg'
_video_format = 'h264'
_log_file_name = 'camera_log.txt'

_VIDEO_CAPTURE_BUTTON = 15
#_CAPTURE_IMAGE_BUTTON = 
_CAMERA_CONTROL_BUTTON = 14
_CAM_LED = 18
_REC_LED = 4
def recorder_setup():
    # check whether the video and image directory exists. if not create one for each
    if not os.path.isdir(_img_dir):
        os.mkdir(_img_dir)

    if not os.path.isdir(_video_dir):
        os.mkdir(_video_dir)

    # setup GPIO inputs
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(_VIDEO_CAPTURE_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(_CAPTURE_IMAGE_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(_CAMERA_CONTROL_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # create putton press events
    GPIO.add_event_detect(_VIDEO_CAPTURE_BUTTON, GPIO.RISING, bouncetime=500)
    #GPIO.add_event_detect(_CAPTURE_IMAGE_BUTTON, GPIO.FALLING, bouncetime=200)
    GPIO.add_event_detect(_CAMERA_CONTROL_BUTTON, GPIO.RISING, bouncetime=300)

    # take control over system LEDs. Power and OK LEDs
    #os.system("sudo bash -c \"echo none > /sys/class/leds/led0/trigger\"")
    #os.system("sudo bash -c \"echo none > /sys/class/leds/led1/trigger\"")
    GPIO.setup(_CAM_LED, GPIO.OUT)
    GPIO.setup(_REC_LED, GPIO.OUT)

    # turn off both LEDs initially
    turn_off_rec_led()
    turn_off_camera_led()

def turn_on_camera_led():
    #os.system("sudo bash -c \"echo 1 > /sys/class/leds/led1/brightness\"")
    GPIO.output(_CAM_LED, GPIO.HIGH)

def turn_off_camera_led():
    #os.system("sudo bash -c \"echo 0 > /sys/class/leds/led1/brightness\"")
    GPIO.output(_CAM_LED, GPIO.LOW)

def turn_on_rec_led():
    #os.system("sudo bash -c \"echo 1 > /sys/class/leds/led0/brightness\"")
    GPIO.output(_REC_LED, GPIO.HIGH)

def turn_off_rec_led():
    GPIO.output(_REC_LED, GPIO.LOW)
    #os.system("sudo bash -c \"echo 0 > /sys/class/leds/led0/brightness\"")

def blink_rec_led():
    turn_on_rec_led()
    sleep(0.5)
    turn_off_rec_led()
    sleep(0.5)
    turn_on_rec_led()
    sleep(0.5)
    turn_off_rec_led()
    sleep(0.5)
    turn_on_rec_led()
    sleep(0.5)
    turn_off_rec_led()
    

def recorder_cleanup():
    GPIO.remove_event_detect(_VIDEO_CAPTURE_BUTTON)
    #GPIO.remove_event_detect(_CAPTURE_IMAGE_BUTTON)
    GPIO.remove_event_detect(_CAMERA_CONTROL_BUTTON)
    
    GPIO.cleanup()

def get_next_image_index(img_dir):
    """Get the number of existing videos and images in the current video and image directory
    This is used to decide the next video / image index for the file name.
    """
    cur_indexes = []
    # scan the image directory and get the last index
    for img in os.listdir(img_dir):
        print img
        if(img.split('.')[-1] == _img_format):
            # get the index of the current image
            img_name = img.split('/')[-1].split('.')[0]
            index = int(img_name.split('_')[-1])
            print index
            cur_indexes.append(index)
            print cur_indexes
    if (not cur_indexes):
        return 0
    else:
        last_idx = max(cur_indexes)
        return last_idx + 1

def get_next_video_index(vid_dir):
    cur_indexes = []
    for vid in os.listdir(vid_dir):
        if(vid.split('.')[-1] == _video_format):
            # get the index of the current image
            vid_name = vid.split('/')[-1].split('.')[0]
            index = int(vid_name.split('_')[-1])
            cur_indexes.append(index)
    if ((not cur_indexes )):
        return 0
    else:
        last_idx = max(cur_indexes)
        return last_idx + 1

def img_cam_setup(camera):
    camera.resolution = _img_resolution
    sleep(0.1)

def vid_cam_setup(camera):
    camera.resolution = _video_resolution
    sleep(0.1)

def take_picture(camera, img_idx, res=_img_resolution):
    img_file = _img_dir + _img_prefix + str(img_idx) + '.' + _img_format
    if (res == _img_resolution):
        camera.capture(img_file)
    else:
        camera.capture(img_file, resize=res)
    # Blink LED
    blink_rec_led()
    return img_file

def record_video_blocking(camera, vid_idx, res=_video_resolution, duration=10):
    vid_file = _video_dir + _video_prefix + str(vid_idx) + '.' + _video_format
    vid_cam_setup(camera)
    if (res == _video_resolution):
        camera.start_recording(vid_file)
        turn_on_rec_led()
        camera.wait_recording(duration)
        camera.stop_recording()
        turn_off_rec_led()
    else:
        camera.start_recording(vid_file, resize=res)
        turn_on_rec_led()
        camera.wait_recording(duration)
        camera.stop_recording()
        turn_off_rec_led()

def record_video_nonblocking(camera, vid_idx, res=_video_resolution):
    """This is non blocking video recording method. It starts recording and returns
    The caller must call stop_video_recording method of the camera module to stop the recording
    """
    vid_file = _video_dir + _video_prefix + str(vid_idx) + '.' + _video_format
    vid_cam_setup(camera)
    if (res == _video_resolution):
        camera.start_recording(vid_file)
    else:
        camera.start_recording(vid_file, resize=res)
#    camera.start_preview()
    # turn on recording LED
    turn_on_rec_led()
    return vid_file

def stop_video_recording(camera):
    camera.stop_recording()
#    camera.stop_preview()
    # turn off recording LED
    turn_off_rec_led()

def wait_for_button_press():
    btn = -1
    while True:
        # assuming all these calls are non-blocking. If they are blocking
        # need to think of callback way of registering button press.
        sleep(0.2)
        if(GPIO.event_detected(_CAMERA_CONTROL_BUTTON)):
            btn = _CAMERA_CONTROL_BUTTON
            break
        #elif(GPIO.event_detected(_CAPTURE_IMAGE_BUTTON)):
        #    btn = _CAPTURE_IMAGE_BUTTON
        #    break
        elif(GPIO.event_detected(_VIDEO_CAPTURE_BUTTON)):
            btn = _VIDEO_CAPTURE_BUTTON
            print 'Video button pressed'
            if (GPIO.input(_VIDEO_CAPTURE_BUTTON) == True):
            #sleep(1)
                break
    return btn

if __name__=='__main__':
    # check if the storage directories exists. If not, create
    recorder_setup()

    # open the log file
    log_file = open(_log_file_name, 'wb+')


    # get the next image and video index for the file name
    next_img_index = get_next_image_index(_img_dir)
    next_vid_index = get_next_video_index(_video_dir)
    rec_state = 'IDLE'
    cam_state = 'OFF'
    try:
        while True:
           # wait for any button press
           # rec_state = 'IDLE'
           btn = wait_for_button_press()
           if (btn == _CAMERA_CONTROL_BUTTON):
               if (cam_state == 'OFF'):
                   # open the camera
                   camera = picamera.PiCamera()
                   cam_state = 'ON'
                   log_file.write('Camera opened\n')
                   # turn on CAM on LED
                   turn_on_camera_led()
               else:
                   camera.close()
                   cam_state = 'OFF'
                   # turn off camera ON LED
                   turn_off_camera_led()
                   log_file.write('Camera closed\n')
           elif (btn == _VIDEO_CAPTURE_BUTTON):
               if (cam_state == 'ON'):
                   if (rec_state == 'REC'):
                       # stop recording
                       stop_video_recording(camera)
                       rec_state = 'IDLE'
                       next_vid_index += 1
                       log_file.write('Recorded video :{:s}\n'.format(vid_file))
                   else:
                       sleep(1.5)
                       if (GPIO.input(_VIDEO_CAPTURE_BUTTON) == True):
                           vid_file = record_video_nonblocking(camera, next_vid_index)
                           rec_state = 'REC'
                           log_file.write('Started recording:{:s}\n'.format(vid_file))
                       else:
                           rec_state = 'CAP'
                           pic = take_picture(camera, next_img_index)
                           next_img_index += 1
                           log_file.write('Picture taken:{:s}\n'.format(pic))
                           rec_state = 'IDLE'
               else:
                   log_file.write('Record button pressed when camera of OFF\n')
           else:
               log_file.write('Unknown button pressed\n')
    finally:
        if (cam_state == 'ON'):
            camera.close()
            log_file.write('Camera closed\n')

        recorder_cleanup()
        log_file.write('Cleaned up the recorder. Exiting\n')
        log_file.close()
