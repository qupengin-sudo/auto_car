import cv2
import numpy as np

def get_lane_offset(img):
    # 1. 影像預處理
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 2. 核心優化：頂帽運算 (針對 320x240 加大 Kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)
    
    # 3. 自動門檻 (Otsu's Binarization)
    _, thresh = cv2.threshold(tophat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # 4. 閉運算補洞
    weld_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, weld_kernel)
    
    # 5. ROI 遮罩範圍設定
    # 頂部 0.60，底部 1.0 (涵蓋至螢幕最下方)
    height, width = thresh.shape
    mask_roi = np.zeros_like(thresh)
    roi_vertices = np.array([[(0, height), (0, int(height * 0.60)), 
                              (width, int(height * 0.60)), (width, height)]], dtype=np.int32)
    cv2.fillPoly(mask_roi, roi_vertices, 255)
    final_mask = cv2.bitwise_and(thresh, mask_roi)

    # 6. 輪廓分析
    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    left_lines_x, right_lines_x = [], []
    frame_center = width // 2
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 面積門檻 600
        if area > 600: 
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            aspect_ratio = max(w, h) / (min(w, h) + 0.1)
            
            # 長寬比判定
            if aspect_ratio > 1.5: 
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    
                    # 尋找底部錨點
                    bottommost = cnt[cnt[:,:,1].argmax()][0]
                    bx = bottommost[0] 
                    
                    # 繪製視覺反饋 (保留繪製功能)
                    if bx < frame_center:
                        left_lines_x.append(cx)
                        cv2.drawContours(img, [cnt], -1, (0, 255, 255), 3) # 黃線
                        cv2.circle(img, tuple(bottommost), 8, (255, 255, 0), -1)
                    else:
                        right_lines_x.append(cx)
                        cv2.drawContours(img, [cnt], -1, (255, 0, 255), 3) # 紫線
                        cv2.circle(img, tuple(bottommost), 8, (255, 255, 0), -1)

    # 7. 計算偏差與狀態
    left_avg = int(np.mean(left_lines_x)) if left_lines_x else None
    right_avg = int(np.mean(right_lines_x)) if right_lines_x else None
    
    line_state = 'none'
    if left_avg and right_avg: line_state = 'both'
    elif left_avg: line_state = 'only_left'
    elif right_avg: line_state = 'only_right'

    lane_center = frame_center
    if line_state == 'both': 
        lane_center = (left_avg + right_avg) // 2
    elif line_state == 'only_left': 
        lane_center = left_avg + 90 
    elif line_state == 'only_right': 
        lane_center = right_avg - 90
    else: 
        return None, 'none', None, img
        
    offset = lane_center - frame_center
    
    # 撞線保護判定
    danger_state = None
    if left_avg and (frame_center - left_avg) < 36: danger_state = 'hit_left'
    elif right_avg and (right_avg - frame_center) < 36: danger_state = 'hit_right'
        
    return offset, line_state, danger_state, img
