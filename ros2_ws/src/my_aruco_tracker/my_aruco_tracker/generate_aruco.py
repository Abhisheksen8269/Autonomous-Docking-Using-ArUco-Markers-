import cv2
import cv2.aruco as aruco

# Define parameters
marker_id = 50 
marker_dictionary = aruco.DICT_5X5_250
side_pixels = 500 # Size of the output image in pixels

# 1. Get the dictionary
aruco_dict = aruco.getPredefinedDictionary(marker_dictionary)

# 2. Generate the marker
marker_image = aruco.generateImage(aruco_dict, marker_id, side_pixels)

# 3. Save the image (Important: Save as PNG for transparency/quality)
filename = f"aruco_{marker_id}.png"
cv2.imwrite(filename, marker_image)

print(f"Generated ArUco marker ID {marker_id} and saved as {filename}")