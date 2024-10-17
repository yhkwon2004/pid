import time
import matplotlib.pyplot as plt
import numpy as np


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.prev_error = 0
        self.integral = 0

    def update(self, measured_value):
        error = self.setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error

        return output


def auto_tune_pid(Kp, setpoint, measured_value):
    # Ziegler-Nichols 방법에 따른 자동 조정
    Ku = 0.0
    Tu = 0.0
    oscillation_detected = False

    # 시스템 반응 테스트
    while not oscillation_detected:
        control_signal = Kp * (setpoint - measured_value)
        measured_value += control_signal * 0.1  # 시스템 응답
        time.sleep(0.1)

        # 여기에서 조건을 설정하여 진동 발생 시 break
        if measured_value > setpoint + 1.0:
            Ku = Kp
            oscillation_detected = True

    # Ziegler-Nichols 튜닝 상수 계산
    Kp = 0.6 * Ku
    Ki = 2 * Kp / Tu
    Kd = Kp * Tu / 8
    return Kp, Ki, Kd


def simulate_system(pid, initial_value, time_steps):
    measured_value = initial_value
    output_vals = []

    for t in range(time_steps):
        control_signal = pid.update(measured_value)
        measured_value += control_signal * 0.1  # 시스템 동역학 반영
        output_vals.append(measured_value)
        time.sleep(0.1)

    return output_vals


# 기본값 설정
Kp = 1.0  # 시작 값
setpoint = float(input("목표 값을 입력하세요: "))
initial_value = float(input("초기 측정 값을 입력하세요: "))
time_steps = int(input("시뮬레이션할 시간 스텝 수를 입력하세요: "))

# 자동 조정 PID 매개 변수 산출
Kp, Ki, Kd = auto_tune_pid(Kp, setpoint, initial_value)

# PID 제어기 초기화
pid = PID(Kp, Ki, Kd, setpoint)

# 시뮬레이션 실행
output_vals = simulate_system(pid, initial_value, time_steps)

# 결과 시각화
plt.plot(output_vals, label='Measured Value')
plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.title('PID Control Simulation with Auto-Tuning')
plt.xlabel('Time Steps')
plt.ylabel('Output Value')
plt.legend()
plt.show()
