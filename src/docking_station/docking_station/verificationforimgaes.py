import cv2
import numpy as np

# 1. Load your new calibration data
K = np.load("K_matrix.npy")
D = np.load("D_coeffs.npy")

# 2. Open Camera
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) # Use the same index/backend as before
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 3. Create the Undistortion Map (We compute this ONCE for speed)
# We need the dimensions of the input image. Let's grab one frame to check size.
ret, frame = cap.read()
if not ret:
    print("Failed to grab frame")
    exit()

h, w = frame.shape[:2]

# "new_K" allows us to zoom in/out of the undistorted image.
# Changing the '1' to '0.5' or '0' will zoom out to show black borders (valid pixels).
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=1)

# Create lookup tables for remapping (this is the fast way to undistort video)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 4. Apply the Undistortion
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # 5. Show Side-by-Side
    # Stack images horizontally to compare
    combined = np.hstack((frame, undistorted_img))
    
    cv2.imshow('Original (Curved) vs Undistorted (Straight)', combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()