import numpy as np 
import pyzed.sl as sl 
import cv2
import os 


image_counter = 1 


def process_key_event(zed, key, directory_path):
    global image_counter
    if key == ord('s'): 
        file_name = os.path.join(directory_path, "calibration_image_" + str(image_counter) + ".png") 

        image_sl_left = sl.Mat()
        zed.retrieve_image(image_sl_left, sl.VIEW.LEFT)
        image_cv_left = image_sl_left.get_data()

        image_sl_right = sl.Mat()
        zed.retrieve_image(image_sl_right, sl.VIEW.RIGHT)
        image_cv_right = image_sl_right.get_data()

        #sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)
        cv2.imwrite(file_name, image_cv_right)
        image_counter += 1
        print("Image saved")


def get_images(): 

    cwd = os.getcwd()
    directory_path = os.path.join(cwd, "calibration_images")
    os.makedirs(directory_path, exist_ok=True)

    zed = sl.Camera() 

    input_type = sl.InputType()
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS: 
        print(repr(err))
        zed.close()
        exit(1)

    runtime = sl.RuntimeParameters()

    image_size = zed.get_camera_information().camera_configuration.resolution

    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    print(directory_path)
    pressed_key = ' '
    while pressed_key != ord('q'): 
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS: 
            zed.retrieve_image(image_zed, sl.VIEW.RIGHT, sl.MEM.CPU, image_size)
            image_ocv = image_zed.get_data()

            cv2.imshow("Image", image_ocv)
            pressed_key = cv2.waitKey(5)

            process_key_event(zed, pressed_key, directory_path)

    cv2.destroyAllWindows()
    zed.close()

    return directory_path

get_images()