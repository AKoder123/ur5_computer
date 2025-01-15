import cv2
import cv2.aruco as aruco

# Define the dictionary and marker ID
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
marker_id = 23  # Change as needed
marker_size = 800  # Marker size in pixels for an 8 cm marker

# Generate and save the marker
marker = aruco.drawMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite("aruco_marker_4x4_id23.png", marker)

# Display the marker
cv2.imshow("Marker", marker)
cv2.waitKey(0)
cv2.destroyAllWindows()