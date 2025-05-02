import sys, time
import numpy as np
sys.path.insert(0, "/home/Paul2901/Downloads/apriltag-main/lib")

from picamera2 import Picamera2
import logging
logging.getLogger("picamera2").setLevel(logging.ERROR)
sys.path.insert(1, '../lib')
import camera_from_tag_finder, location, drawing_mira220, time
from camera_from_tag_finder import Detector_mira220


def Run_eachPressKey(tagfinder_obj):
    
    tagfinder_obj.capture_Camera()

    if not tagfinder_obj.getPose():
        print("0.0 0.0 0.0 0.0", flush=True)
        time.sleep(1)
        return 0

    closest_tag = None
    min_distance = float('inf')

    for tag in tagfinder_obj.tag_positions:
        abs_Z = abs(tag['Z'])  # Take absolute value of Z to avoid sign errors
#         print(f"Tag {tag['id']} - Position: X={int(tag['X'])}, Y={int(tag['Y'])}, Z={int(tag['Z'])}", flush=True)

        # Check for the closest tag
        if abs_Z < min_distance:  # Compare based on absolute distance
            min_distance = abs_Z
            closest_tag = tag

    if closest_tag is None:
        print("Error: Unable to determine the closest tag.", flush=True)
        return 0
    
    # Check if quaternion is available
    quaternion = closest_tag.get('Quaternion', (1.0, 0.0, 0.0, 0.0))  # Default value if missing

    # Display the closest tag
#     print(f"==== Closest Tag ({closest_tag['id']}) - X: {int(closest_tag['X'])}, Y: {int(closest_tag['Y'])}, Z: {int(closest_tag['Z'])}", flush=True)
#     print(f"==== Orientation (Yaw, Pitch, Roll): {closest_tag['Yaw']}, {closest_tag['Pitch']}, {closest_tag['Roll']}", flush=True)
#     print(f"==== Quaternion (w, x, y, z): {quaternion}", flush=True)

    print(f"{quaternion}", flush=True)

    # Annotate the image if display is available
    if os.getenv('DISPLAY') is not None:
        draw_obj.img = tagfinder_obj.img
        draw_obj.annotate_Image(10, 30,
                                int(closest_tag['X']), int(closest_tag['Y']), int(closest_tag['Z']),
                                closest_tag['Yaw'], closest_tag['Pitch'], closest_tag['Roll'],
                                quaternion)
        draw_obj.draw_Cube()
        return tagfinder_obj.showImage(tagfinder_obj.img, 2000, 1)
    
    # Headless mode (no display)
    return ord('*')


# ======================== Initialization ===============================

tagfinder_obj = Detector_mira220(0.097)  # Tag size in meters
draw_obj = drawing_mira220.Draw(tagfinder_obj)

keypress = ord('*')  # Arbitrary key

while keypress != ord('a'):
    keypress = Run_eachPressKey(tagfinder_obj)
#     print("Keypress: ", keypress, flush=True)

tagfinder_obj.release_Camera()
tagfinder_obj.destroyAllWindows()
