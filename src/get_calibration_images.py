#import pyzed.sl as sl
import cv2
import os 

image_counter = 1

def get_images():
    global image_counter
    cwd = os.getcwd()
    directory_path = os.path.join(cwd, "calibration_images")
    os.makedirs(directory_path, exist_ok=True)

    cam = cv2.VideoCapture(0)

    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to grab frame")
            break

        cv2.imshow("Image", frame)
        pressed_key = cv2.waitKey(5)

        if pressed_key == ord('q'):
            print("Escape hit, closing")
            break

        if pressed_key == ord('s'):
            file_name = os.path.join(directory_path, "calibration_image_" + str(image_counter) + ".png")
            cv2.imwrite(file_name, frame)
            image_counter += 1

    cam.release()
    cv2.destroyAllWindows()

    return directory_path


if __name__ == "__main__":
    get_images()
