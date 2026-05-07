import cv2
import numpy as np

class TrafficManager:
    def __init__(self, frame_interval=3, stop_y_threshold=70):
        self.frame_interval = frame_interval  
        self.frame_count = 0
        self.current_light = 'none' 
        self.stop_y_threshold = stop_y_threshold

    def check_light(self, frame):
        self.frame_count += 1
        
        if self.frame_count % self.frame_interval != 0:
            return self.current_light 

        # 截取畫面上半部 85%
        height, width = frame.shape[:2]
        crop = frame[0:int(height * 0.85), :]
        
        # 傳入畫面寬度以進行左右對稱判斷
        red_state = self._analyze_color(crop, 'red', width)
        green_state = self._analyze_color(crop, 'green', width)
        
        if red_state == 'close':
            self.current_light = 'red_stop' 
        elif red_state == 'far':
            self.current_light = 'red_far'  
        elif green_state in ['close', 'far']:
            self.current_light = 'green'    
        else:
            self.current_light = 'none'
            
        return self.current_light

    def _analyze_color(self, crop, color, frame_width):
        if crop.size == 0: return 'none'
        
        blurred = cv2.GaussianBlur(crop, (9, 9), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        if color == 'red':
            # 紅色範圍 (包含過曝的偏粉白紅光)
            lower1, upper1 = np.array([0, 50, 80]), np.array([10, 255, 255])
            lower2, upper2 = np.array([160, 50, 80]), np.array([180, 255, 255])
            mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
        elif color == 'green':
            # 嚴格綠色範圍：高亮度 (V>180)，避開背景綠色
            lower, upper = np.array([40, 40, 180]), np.array([90, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        else:
            return 'none'
            
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        state = 'none'
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 面積過濾雜訊
            if 40 < area < 4000:  
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 🌟【鏡像距離邏輯】：判斷左上角或右上角
                # 1. 垂直高度條件：y < 120 (在畫面上半部)
                # 2. 尺寸條件：寬度 w > 12 且 高度 h > 12 (靠得夠近)
                # 3. 水平位置條件：
                #    右側判定: x > (frame_width * 0.5)
                #    左側判定: (x + w) < (frame_width * 0.5)
                
                is_in_top_zone = (y < 120)
                is_large_enough = (w > 12 and h > 12)
                is_right_side = (x > (frame_width * 0.5))
                is_left_side = ((x + w) < (frame_width * 0.5))

                if is_in_top_zone and is_large_enough and (is_right_side or is_left_side):
                    state = 'close' # 進入左側或右側的理想停車區
                    break # 只要有一個符合即觸發停車
                else:
                    if state != 'close': 
                        state = 'far'
                        
        return state
