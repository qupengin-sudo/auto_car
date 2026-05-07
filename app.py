from flask import Flask, render_template, request, Response, jsonify
import cv2
import threading
import time
from picamera2 import Picamera2
from motor_control import LOBOROBOT
from detect import get_lane_offset
from traffic import TrafficManager

app = Flask(__name__)
car = LOBOROBOT()
i2c_lock = threading.Lock()

# --- 參數設定區 ---
MANUAL_SPEED = 40  
AUTO_SPEED = 35    
OFFSET_SPEED = 10  
SHARP_OFFSET_SPEED = 25  

pan_angle = 90
tilt_angle = 40
auto_drive = False
last_offset = 0

# 🌟 紅綠燈鎖死與起步記憶變數
waiting_for_green = False  
last_moving_state = 'forward' 

current_light_state = 'none'
current_offset = 0.0

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": 'RGB888', "size": (320, 240)})
picam2.configure(config)
picam2.start()

# 初始化紅綠燈管理員 (設定為每 5 幀偵測一次，維持畫面順暢)
traffic_mgr = TrafficManager(frame_interval=5)

def gen_frames():
    global auto_drive, last_offset, waiting_for_green, last_moving_state
    global current_light_state, current_offset
    current_state = None  
    
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        # 取得紅綠燈狀態 ('red_stop', 'red_far', 'green', 'none')
        light_state = traffic_mgr.check_light(frame)
        current_light_state = light_state
        
        offset, line_state, danger_state, processed_frame = get_lane_offset(frame)
        if offset is not None:
            current_offset = offset
            
        if auto_drive and car:
            target_state = 'stop'
            
            # --- 0. 嚴格紅綠燈狀態機 ---
            if light_state == 'red_stop':
                waiting_for_green = True # 進入右上角區塊，觸發停車鎖死
                
            elif light_state == 'green':
                waiting_for_green = False # 只有看到綠燈，才會解鎖！
            
            # 只要被鎖死，就強制停車，無視其他循線邏輯
            if waiting_for_green:
                target_state = 'stop'
                            
            # --- 1. 優先處理危險撞線 ---
            elif danger_state == 'hit_left':
                target_state = 'sharp_steer_right'
                last_moving_state = target_state 
            elif danger_state == 'hit_right':
                target_state = 'sharp_steer_left'
                last_moving_state = target_state
            
            # --- 2. 正常車道微調 ---
            elif line_state != 'none' and offset is not None:
                last_offset = offset  
                if last_offset > 18:   
                    target_state = 'steer_right'
                elif last_offset < -18:  
                    target_state = 'steer_left'
                else:                    
                    target_state = 'forward'
                last_moving_state = target_state 
                
            # --- 3. 霸道丟線補償 ---
            elif line_state == 'none':
                target_state = last_moving_state

            # --- 硬體執行層 ---
            if target_state != current_state:
                with i2c_lock:
                    if target_state == 'stop':
                        car.t_stop(0)
                    elif target_state == 'sharp_steer_right':
                        car.move_with_offset(AUTO_SPEED, SHARP_OFFSET_SPEED, -SHARP_OFFSET_SPEED, 0)
                    elif target_state == 'sharp_steer_left':
                        car.move_with_offset(AUTO_SPEED, -SHARP_OFFSET_SPEED, SHARP_OFFSET_SPEED, 0)
                    elif target_state == 'steer_right':
                        car.move_with_offset(AUTO_SPEED, OFFSET_SPEED, -OFFSET_SPEED, 0)
                    elif target_state == 'steer_left':
                        car.move_with_offset(AUTO_SPEED, -OFFSET_SPEED, OFFSET_SPEED, 0)
                    elif target_state == 'forward':
                        car.MotorRun(0, 'forward', AUTO_SPEED); time.sleep(0.01)
                        car.MotorRun(1, 'forward', AUTO_SPEED); time.sleep(0.01)
                        car.MotorRun(2, 'forward', AUTO_SPEED); time.sleep(0.01)
                        car.MotorRun(3, 'forward', AUTO_SPEED)
                current_state = target_state 

        # 影像傳輸 (畫面完全乾淨，不繪製任何字體)
        ret, buffer = cv2.imencode('.jpg', processed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        time.sleep(0.1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_status')
def get_status():
    global current_light_state, current_offset, waiting_for_green
    return jsonify({
        'light': current_light_state,
        'offset': round(current_offset, 1),
        'waiting': waiting_for_green
    })

@app.route('/auto_control', methods=['POST'])
def auto_control():
    global auto_drive, waiting_for_green
    mode = request.form.get('mode')
    if mode == 'start': 
        auto_drive = True
        waiting_for_green = False 
    elif mode == 'stop':
        auto_drive = False
        with i2c_lock: car.t_stop(0)
    return 'OK'

@app.route('/control', methods=['POST'])
def control():
    global pan_angle, tilt_angle
    action = request.form.get('action')
    
    if car and not auto_drive:
        with i2c_lock:
            try:
                if action == 'forward': 
                    car.MotorRun(0, 'forward', MANUAL_SPEED); time.sleep(0.01)
                    car.MotorRun(1, 'forward', MANUAL_SPEED); time.sleep(0.01)
                    car.MotorRun(2, 'forward', MANUAL_SPEED); time.sleep(0.01)
                    car.MotorRun(3, 'forward', MANUAL_SPEED)
                elif action == 'backward': 
                    car.MotorRun(0, 'backward', MANUAL_SPEED); time.sleep(0.01)
                    car.MotorRun(1, 'backward', MANUAL_SPEED); time.sleep(0.01)
                    car.MotorRun(2, 'backward', MANUAL_SPEED); time.sleep(0.01)
                    car.MotorRun(3, 'backward', MANUAL_SPEED)
                elif action == 'left': car.turnLeft(speed=MANUAL_SPEED, t_time=0)
                elif action == 'right': car.turnRight(speed=MANUAL_SPEED, t_time=0)
                elif action == 'left_slide': car.moveLeft(speed=MANUAL_SPEED, t_time=0)
                elif action == 'right_slide': car.moveRight(speed=MANUAL_SPEED, t_time=0)
                elif action == 'stop': car.t_stop(t_time=0)
                
                elif action == 'cam_up':
                    tilt_angle = max(0, tilt_angle - 5)
                    car.set_servo_angle(9, tilt_angle, 0)
                elif action == 'cam_down':
                    tilt_angle = min(80, tilt_angle + 5)
                    car.set_servo_angle(9, tilt_angle, 0)
                elif action == 'cam_left':
                    pan_angle = min(150, pan_angle + 5)
                    car.set_servo_angle(10, pan_angle, 0)
                elif action == 'cam_right':
                    pan_angle = max(30, pan_angle - 5)
                    car.set_servo_angle(10, pan_angle, 0)
                elif action == 'cam_center':
                    pan_angle, tilt_angle = 77,0
                    car.set_servo_angle(10, pan_angle, 0)
                    car.set_servo_angle(9, tilt_angle, 0)
            except Exception as e:
                print(f"硬體通訊例外: {e}")
    return 'OK'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=6255, threaded=True)
