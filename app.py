from flask import Flask, render_template, request, Response
import cv2
import threading
import time
from picamera2 import Picamera2
from motor_control import LOBOROBOT
from detect import get_lane_offset

app = Flask(__name__)
car = LOBOROBOT()
i2c_lock = threading.Lock()

# --- 參數設定區 ---
MANUAL_SPEED = 40  # 手動模式速度 
AUTO_SPEED = 35    # 自動駕駛基礎速度 
OFFSET_SPEED = 10  # 一般轉向時的微調差速
SHARP_OFFSET_SPEED = 25  # 新增：急彎模式的大幅差速補償量

# --- 狀態變數 ---
pan_angle = 90
tilt_angle = 40
auto_drive = False
last_offset = 0

# 初始化相機
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": 'RGB888', "size": (160, 120)})
picam2.configure(config)
picam2.start()
def gen_frames():
    global auto_drive, last_offset
    current_state = None  
    
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        offset, line_state, danger_state, processed_frame = get_lane_offset(frame)
            
        if auto_drive and car:
            target_state = 'stop'
            
            # --- 1. 優先處理危險撞線 ---
            if danger_state == 'hit_left':
                target_state = 'sharp_steer_right'
            elif danger_state == 'hit_right':
                target_state = 'sharp_steer_left'
            
            # --- 2. 正常車道微調 (看見雙線或單線) ---
            elif line_state != 'none' and offset is not None:
                last_offset = offset  # 更新最後一次的有效偏差值
                if last_offset > 9:   
                    target_state = 'steer_right'
                elif last_offset < -9:  
                    target_state = 'steer_left'
                else:                    
                    target_state = 'forward'
                
            # --- 3. 霸道丟線補償：無限期維持前動作直到看到線 ---
            elif line_state == 'none':
                # 只要前一個動作是前進或轉彎，就無限期維持該動作
                valid_states = ['sharp_steer_right', 'sharp_steer_left', 'steer_right', 'steer_left', 'forward']
                if current_state in valid_states:
                    target_state = current_state
                    # [已移除 cv2.putText 文字渲染，釋放 CPU 效能]
                else:
                    target_state = 'stop'  # 除非一開始就沒看過線，才會停車

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

        # 影像傳輸 (保留 50% 壓縮維持系統低負載)
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

@app.route('/auto_control', methods=['POST'])
def auto_control():
    global auto_drive
    mode = request.form.get('mode')
    if mode == 'start': 
        auto_drive = True
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
                
                # 鏡頭平滑控制
                elif action == 'cam_up':
                    tilt_angle = max(20, tilt_angle - 5)
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
                    pan_angle, tilt_angle = 90, 40
                    car.set_servo_angle(10, pan_angle, 0)
                    car.set_servo_angle(9, tilt_angle, 0)
            except Exception as e:
                print(f"硬體通訊例外: {e}")
    return 'OK'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=6255, threaded=True)
