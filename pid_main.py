import time
import matplotlib.pyplot as plt


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


# PID 제어기 초기화
pid = PID(Kp=2.0, Ki=1.0, Kd=0.5, setpoint=100)

# 시뮬레이션 변수
measured_value = 20  # 초기 측정 값
time_steps = []
output_vals = []

# 시뮬레이션 진행
for t in range(100):
    control_signal = pid.update(measured_value)
    measured_value += control_signal * 0.1
    time_steps.append(t)
    output_vals.append(measured_value)
    time.sleep(0.1)  # 100ms 딜레이

# 결과 시각화
plt.plot(time_steps, output_vals, label='Measured Value')
plt.axhline(y=pid.setpoint, color='r', linestyle='--', label='Setpoint')
plt.title('PID Control Simulation')
plt.xlabel('Time')
plt.ylabel('Output Value')
plt.legend()
plt.show()
