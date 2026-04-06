import time
from motor_control import LOBOROBOT

def test_all_motors():
    try:
        robot = LOBOROBOT()
        print("✅ 成功初始化 LOBOROBOT")
    except Exception as e:
        print(f"❌ 初始化失敗: {e}")
        return

    motor_names = ["0: 左前輪 (PWMA)", "1: 右前輪 (PWMB)", "2: 左後輪 (PWMC)", "3: 右後輪 (PWMD)"]
    test_speed = 60  # 測試速度建議設在 50~70 之間

    print("\n--- 開始逐一測試馬達 ---")
    for i in range(4):
        print(f"正在測試 -> {motor_names[i]}...")
        
        # 讓該顆馬達前進 2 秒
        robot.MotorRun(i, 'forward', test_speed)
        time.sleep(2)
        
        # 停止該顆馬達
        robot.MotorStop(i)
        print(f"已停止 -> {motor_names[i]}")
        time.sleep(1)

    print("\n--- 測試結束 ---")
    print("如果其中一顆沒動，請檢查該馬達的接線與驅動版插槽。")

if __name__ == "__main__":
    test_all_motors()
