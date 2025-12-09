import cv2
import numpy as np
import os
import glob

# ============================================
# CONFIGURATION
# ============================================
CHECKERBOARD = (9, 6)  # Ensure this matches your inner corners
SQUARE_SIZE = 20       # mm (optional, affects scale only)
IMG_DIR = "calibration_images"
CAM_INDEX = 1          # Check if this should be 0 or 1
# ============================================

def capture_images():
    if not os.path.exists(IMG_DIR):
        os.makedirs(IMG_DIR)
    
    # Use DirectShow to prevent Windows MSMF errors
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print(f"Error: Could not open camera {CAM_INDEX}.")
        return

    print("Press 's' to save an image, 'q' to finish capturing.")
    print("TIP: Take photos where the board is TILTED. Flat photos cause errors.")
    count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret: break

        display = frame.copy()
        cv2.putText(display, f"Saved: {count}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Capture Calibration Images', display)

        key = cv2.waitKey(1)
        if key == ord('s'):
            filename = f"{IMG_DIR}/img_{count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")
            count += 1
        elif key == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

def run_calibration():
    print("--- Starting Detection ---")
    
    # Prepare object points (0,0,0), (1,0,0), ...
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * SQUARE_SIZE

    objpoints = [] 
    imgpoints = [] 
    valid_filenames = [] # <--- NEW: Keep track of which file corresponds to which index

    images = glob.glob(f'{IMG_DIR}/*.jpg')
    images.sort() # Sort to ensure consistent ordering

    for fname in images:
        img = cv2.imread(fname)
        if img is None: continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find corners
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(
                gray, corners, (3, 3), (-1, -1), 
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            )
            imgpoints.append(corners2)
            valid_filenames.append(fname)
            print(f"  [OK] {fname}")
        else:
            print(f"  [FAIL] {fname} - No corners found")

    print(f"\nFound {len(valid_filenames)} valid images.")
    
    if len(valid_filenames) == 0:
        print("Error: No corners found in any images.")
        return

    # --- DEBUGGING TABLE ---
    print("\n" + "="*40)
    print("      IMAGE INDEX MAP (DEBUG)      ")
    print("="*40)
    print("If it crashes at 'input array X', look for [X] below:")
    for i, name in enumerate(valid_filenames):
        print(f"  [{i}] -> {name}")
    print("="*40 + "\n")

    print("Calibrating... (this might take a moment)")
    
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))]

    try:
        rms, _, _, _, _ = cv2.fisheye.calibrate(
            objpoints, imgpoints, gray.shape[::-1],
            K, D, rvecs, tvecs, calibration_flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )

        print(f"\nSUCCESS!")
        print(f"Calibration RMS error: {rms}")
        print("K Matrix:\n", K)
        print("D Coefficients:\n", D)

        np.save("K_matrix.npy", K)
        np.save("D_coeffs.npy", D)
        print("Saved calibration files.")

    except cv2.error as e:
        print("\n!!! CALIBRATION CRASHED !!!")
        print("Look at the error message above.")
        print("If it says 'input array 23', go to the table above, find [23], and DELETE that file.")
        print("Reason: That specific image likely has a 'bad angle' (too flat or edge-on).")
        print(f"OpenCV Error Details: {e}")

if __name__ == "__main__":
    choice = input("Type 'c' to Capture, 'r' to Run: ")
    if choice.lower() == 'c': capture_images()
    elif choice.lower() == 'r': run_calibration()