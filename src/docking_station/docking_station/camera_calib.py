import cv2
import numpy as np
import os

def nothing(x):
    pass

def apply_distance_correction(camera_dist_m):
    return camera_dist_m

def get_contour_details(frame_gray, contour):
    """
    Returns the max brightness and bounding box of a contour.
    Used to distinguish real LEDs (brightness ~255) from reflections.
    """
    x, y, w, h = cv2.boundingRect(contour)
    
    # Create a mask for just this contour to measure its brightness
    # (We use a small ROI to speed up processing)
    roi = frame_gray[y:y+h, x:x+w]
    
    # We just check the max value in this box. 
    # Real LEDs usually hit 254-255. Reflections usually stay below 230.
    min_val, max_val, _, _ = cv2.minMaxLoc(roi)
    
    return max_val, (x, y, w, h)

def main():
    # --- 1. LOAD CALIBRATION ---
    if not os.path.exists("K_matrix.npy") or not os.path.exists("D_coeffs.npy"):
        print("ERROR: Calibration files missing!")
        # For testing purposes without files, uncomment the next two lines:
        # K = np.eye(3); D = np.zeros(4)
        return

    K = np.load("K_matrix.npy")
    D = np.load("D_coeffs.npy")
    
    # --- 2. OPEN CAMERA ---
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

    cv2.namedWindow("Main Vision")
    cv2.namedWindow("Zoomed ROI") 

    # --- 3. SLIDERS ---
    # Coarse Thresh: Lower this to catch LEDs even if they are dim
    cv2.createTrackbar("Coarse Thresh", "Main Vision", 100, 255, nothing)
    
    # Min Brightness: Crucial for ignoring reflections. 
    # Set to 240+ to only see actual light sources.
    cv2.createTrackbar("Min Brightness", "Main Vision", 240, 255, nothing)

    cv2.createTrackbar("Zoom Scale", "Main Vision", 8, 16, nothing)
    
    # --- ZOOMED ROI SLIDERS ---
    cv2.createTrackbar("Fine Thresh", "Zoomed ROI", 200, 255, nothing)
    # NEW: Split Aggression (0 = off, 3-5 = normal, 10 = aggressive)
    cv2.createTrackbar("Split Aggression", "Zoomed ROI", 3, 10, nothing) 
    # Keeping old erosion slider just in case, though the smart splitter often replaces it
    cv2.createTrackbar("Erosion", "Zoomed ROI", 0, 10, nothing) 

    cv2.createTrackbar("LED Sep (mm)", "Main Vision", 150, 500, nothing)
    cv2.createTrackbar("Cam X (cm)", "Main Vision", 300, 500, nothing)
    cv2.createTrackbar("Cam Y (cm)", "Main Vision", 0, 500, nothing)
    cv2.createTrackbar("Exposure (-)", "Main Vision", 10, 100, nothing)
    
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    last_exposure = -999 

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret: break
        
        height, width = frame.shape[:2]
        
        # UI Updates
        coarse_thresh = cv2.getTrackbarPos("Coarse Thresh", "Main Vision")
        min_brightness_cutoff = cv2.getTrackbarPos("Min Brightness", "Main Vision")
        zoom_scale = cv2.getTrackbarPos("Zoom Scale", "Main Vision")
        if zoom_scale < 1: zoom_scale = 1
        
        fine_thresh = cv2.getTrackbarPos("Fine Thresh", "Zoomed ROI")
        split_aggression = cv2.getTrackbarPos("Split Aggression", "Zoomed ROI")
        erosion_iter = cv2.getTrackbarPos("Erosion", "Zoomed ROI")
        
        led_sep_mm = cv2.getTrackbarPos("LED Sep (mm)", "Main Vision")
        cam_x_cm = cv2.getTrackbarPos("Cam X (cm)", "Main Vision")
        cam_y_cm = cv2.getTrackbarPos("Cam Y (cm)", "Main Vision")
        exp_slider = cv2.getTrackbarPos("Exposure (-)", "Main Vision")
        
        if exp_slider < 1: exp_slider = 1
        target_exposure = -exp_slider
        if target_exposure != last_exposure:
            cap.set(cv2.CAP_PROP_EXPOSURE, target_exposure)
            last_exposure = target_exposure

        display_frame = frame.copy()
        
        # --- STEP 1: COARSE SEARCH (Smart Cluster) ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask_coarse = cv2.threshold(gray, coarse_thresh, 255, cv2.THRESH_BINARY)
        contours_coarse, _ = cv2.findContours(mask_coarse, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # List to store valid LED boxes
        valid_boxes = []

        if contours_coarse:
            for cnt in contours_coarse:
                # Filter by Area (very coarse check)
                if cv2.contourArea(cnt) < 1: continue
                
                # Filter by Brightness (The "Reflection Killer")
                max_val, rect = get_contour_details(gray, cnt)
                
                # Only keep blobs that are BRIGHT (e.g. > 240)
                if max_val >= min_brightness_cutoff:
                    valid_boxes.append(rect)
                    # Draw blue box around "Valid" candidates on main screen
                    x,y,w,h = rect
                    cv2.rectangle(display_frame, (x,y), (x+w, y+h), (255,0,0), 1)

        # Variables for final calc
        pt1_final = None
        pt2_final = None

        # --- STEP 2: CALCULATE UNION BOX ---
        # If we found valid LEDs, create a box that contains ALL of them
        if len(valid_boxes) > 0:
            # Find min_x, min_y, max_x, max_y across all boxes
            min_x = min([b[0] for b in valid_boxes])
            min_y = min([b[1] for b in valid_boxes])
            max_x = max([b[0] + b[2] for b in valid_boxes])
            max_y = max([b[1] + b[3] for b in valid_boxes])
            
            # Add generous padding so we don't clip edges
            padding = 30
            roi_x1 = max(0, min_x - padding)
            roi_y1 = max(0, min_y - padding)
            roi_x2 = min(width, max_x + padding)
            roi_y2 = min(height, max_y + padding)
            
            # Draw the "Zoom Target" on main screen (Green Box)
            cv2.rectangle(display_frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 2)

            # --- STEP 3: ZOOM & PROCESS ---
            roi_gray = gray[roi_y1:roi_y2, roi_x1:roi_x2]
            
            if roi_gray.size > 0:
                # A. Upscale
                roi_zoomed = cv2.resize(roi_gray, None, fx=zoom_scale, fy=zoom_scale, interpolation=cv2.INTER_LINEAR)
                
                # B. CONTRAST BOOST (Squaring Filter)
                # Normalize to 0-255 first
                roi_norm = cv2.normalize(roi_zoomed, None, 0, 255, cv2.NORM_MINMAX)
                # Square the values to crush grey glare into black
                roi_float = roi_norm.astype(float)
                roi_enhanced = (roi_float ** 2) / 255.0
                roi_enhanced = roi_enhanced.astype(np.uint8)
                
                # C. Fine Threshold on Enhanced Image
                _, mask_fine = cv2.threshold(roi_enhanced, fine_thresh, 255, cv2.THRESH_BINARY)
                
                # Manual Erosion (if user wants it globally)
                if erosion_iter > 0:
                    kernel = np.ones((3,3), np.uint8)
                    mask_fine = cv2.erode(mask_fine, kernel, iterations=erosion_iter)

                # D. SMART SPLITTER (Auto-Erosion Loop)
                # If we see 1 blob, we try to split it into 2
                contours_fine, _ = cv2.findContours(mask_fine.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_fine = [c for c in contours_fine if cv2.contourArea(c) > (2 * zoom_scale)] # Filter noise
                
                if len(contours_fine) == 1 and split_aggression > 0:
                    temp_mask = mask_fine.copy()
                    kernel_split = np.ones((3,3), np.uint8)
                    
                    # Try eroding up to 'split_aggression' times
                    for _ in range(split_aggression + 2):
                        temp_mask = cv2.erode(temp_mask, kernel_split, iterations=1)
                        temp_cnts, _ = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        temp_cnts = [c for c in temp_cnts if cv2.contourArea(c) > (2 * zoom_scale)]
                        
                        if len(temp_cnts) >= 2:
                            # We successfully split them!
                            mask_fine = temp_mask
                            contours_fine = temp_cnts
                            break

                # Viz (Show the Enhanced/Squared image in the zoom window)
                roi_color = cv2.cvtColor(roi_enhanced, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(roi_color, contours_fine, -1, (0,255,0), 1)
                cv2.imshow("Zoomed ROI", roi_color)

                # Map Coordinates Back
                centroids_original = []
                # Re-check count (after potential splitting)
                if len(contours_fine) >= 2:
                    for c in contours_fine:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cz_x = M["m10"] / M["m00"]
                            cz_y = M["m01"] / M["m00"]
                            
                            # Transform: Original = Offset + (Zoomed / Scale)
                            orig_x = roi_x1 + (cz_x / zoom_scale)
                            orig_y = roi_y1 + (cz_y / zoom_scale)
                            
                            centroids_original.append((orig_x, orig_y))
                            
                            # Draw Red Dots on Main Frame
                            cv2.circle(display_frame, (int(orig_x), int(orig_y)), 3, (0, 0, 255), -1)

                    # Find pair with max distance
                    if len(centroids_original) >= 2:
                        max_d = -1
                        for i in range(len(centroids_original)):
                            for j in range(i+1, len(centroids_original)):
                                p1 = np.array(centroids_original[i])
                                p2 = np.array(centroids_original[j])
                                d = np.linalg.norm(p1 - p2)
                                if d > max_d:
                                    max_d = d
                                    pt1_final = p1
                                    pt2_final = p2

        # --- STEP 4: PHYSICS & DRAWING ---
        if pt1_final is not None and pt2_final is not None:
             points_raw = np.array([[pt1_final], [pt2_final]], dtype=np.float64)
             points_undist = cv2.fisheye.undistortPoints(points_raw, K, D)
             
             p1n = points_undist[0][0]
             p2n = points_undist[1][0]
             
             ray1 = np.array([p1n[0], p1n[1], 1.0])
             ray2 = np.array([p2n[0], p2n[1], 1.0])
             
             dot = np.dot(ray1, ray2)
             norm = np.linalg.norm(ray1) * np.linalg.norm(ray2)
             theta = np.arccos(np.clip(dot/norm, -1, 1))
             
             if theta > 1e-5:
                 depth = ((led_sep_mm/1000.0) / 2.0) / np.tan(theta/2.0)
                 depth = apply_distance_correction(depth)
                 
                 avg_ray_x = (ray1[0] + ray2[0]) / 2.0
                 lat_dist = depth * avg_ray_x
                 
                 wx = (cam_x_cm/100.0) - depth
                 wy = (cam_y_cm/100.0) + lat_dist
                 
                 p1i = (int(pt1_final[0]), int(pt1_final[1]))
                 p2i = (int(pt2_final[0]), int(pt2_final[1]))
                 cv2.line(display_frame, p1i, p2i, (0,255,255), 2)
                 
                 cv2.putText(display_frame, f"Depth: {depth:.3f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                 cv2.putText(display_frame, f"X: {wx:.2f} Y: {wy:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        else:
            cv2.putText(display_frame, "Searching...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            if 'roi_zoomed' not in locals():
                dummy = np.zeros((200, 200, 3), dtype=np.uint8)
                cv2.imshow("Zoomed ROI", dummy)

        cv2.imshow("Main Vision", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()