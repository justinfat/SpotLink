import cv2
import sys

def webcam_capture(cam_id):
    cap = cv2.VideoCapture(cam_id)

    print('Video width:',cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print('Video height:',cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print('Video fps:',cap.get(cv2.CAP_PROP_FPS))

    #if cap.isOpened() == False:
    #    cap.open()

    while cap.isOpened():
        ret, frame = cap.read()

        # when read sucessfully, ret = True
        frame_count = cap.get(cv2.CAP_PROP_POS_FRAMES)
        if not ret:
            print('unable to read the video...')
            break

        cv2.imshow('webcam capture', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cap.release()
    cv2.destoryAllWindows()

def main(argv=None):
    if argv is None:
        argv = sys.argv
    print(argv)
    print('Opencv version:', cv2.__version__)

    webcam_capture(0)

if __name__ == '__main__':
    sys.exit(main())