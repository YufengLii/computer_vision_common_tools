import sys
import numpy as np
import pyzed.sl as sl
import cv2
import os

prefix_image_Left = "mm_left_"
prefix_image_Right = "mm_right_"

prefix_depth = "mm_depth_"
path = "./zed_data/"


def save_image_and_depth(zed, image_left, image_right, image_depth, scene_num, path, current_distance, image_num):

    filename_left = path + current_distance + prefix_image_Left + str(scene_num) + '_' + str(image_num) + '.jpeg'
    filename_right = path + current_distance + prefix_image_Right + str(scene_num) + '_' + str(image_num)+ '.jpeg'
    filename_depth = path + current_distance + prefix_depth + str(scene_num) + '_' + str(image_num)+ '.png'

    cv2.imwrite(filename_right, image_right)
    cv2.imwrite(filename_left, image_left)
    cv2.imwrite(filename_depth, image_depth)

def main() :

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    input_type = sl.InputType()
    if len(sys.argv) >= 2 :
        input_type.set_from_svo_file(sys.argv[1])
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # Display help in console
    # print_help()

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width /2
    image_size.height = image_size.height /2

    # Declare your sl.Mat matrices
    image_zed_L = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    image_zed_R = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    key = ' '
    frame_count = 0
    scene_num = 0
    while key != 'q' :

        if (frame_count % 300) == 0 :
            
            current_distance = input("Please Input scene distance(for example 600): ")
            os.mkdir( path + current_distance , 0755 )
        if (frame_count % 30) == 0 :
            scene_num = scene_num + 1

            print("Please change scene!!!")
            while True :
                scene_changed = input("scene has changed(y/n): ")
                if scene_changed == 'y':
                    break
                else :
                    print("Please change scene!!!")

        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image, depth image in the half-resolution
            zed.retrieve_image(image_zed_L, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            zed.retrieve_image(image_zed_R, sl.VIEW.RIGHT, sl.MEM.CPU, image_size)
            zed.retrieve_image(depth_image, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
            # Retrieve the RGBA point cloud in half resolution
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv_L = image_zed_L.get_data()
            image_ocv_R = image_zed_R.get_data()
            depth_image_ocv = depth_image.get_data()

            cv2.imshow("Image_L", image_ocv_L)
            cv2.imshow("Image_R", image_ocv_R)
            cv2.imshow("Depth", depth_image_ocv)            
            frame_count += 1

            save_image_and_depth(zed, image_ocv_L, image_ocv_R, depth_image_ocv, scene_num, path + current_distance + '/', current_distance, frame_count % 30)

            if scene_num == 10 :
                scene_num = 0
            key = cv2.waitKey(10)

            # process_key_event(zed, key)

    cv2.destroyAllWindows()
    zed.close()

    print("\nFINISH")