import time
import sys
import struct

# MicroPython provides these modules only on the ESP32.
try:
    from machine import Pin, PWM  # pyright: ignore[reportMissingImports]
    import bluetooth  # pyright: ignore[reportMissingImports]
except ImportError:
    HARDWARE_AVAILABLE = False
else:
    HARDWARE_AVAILABLE = (
        sys.implementation.name == "micropython" and sys.platform == "esp32"
    )

BLE_NAME = "Robot-S3"
PACKET_MAGIC = 0xA7
PACKET_FORMAT = "<BhhhH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
CONTROL_TIMEOUT_MS = 500

BUTTON_LIFT = 1 << 0
BUTTON_LOWER = 1 << 1
BUTTON_GRIP = 1 << 2
BUTTON_RELEASE = 1 << 3
BUTTON_LIGHT = 1 << 4
BUTTON_PICK = 1 << 5
BUTTON_DROP = 1 << 6
BUTTON_RESET = 1 << 7

_BLE_IRQ_CENTRAL_CONNECT = 1
_BLE_IRQ_CENTRAL_DISCONNECT = 2
_BLE_IRQ_GATTS_WRITE = 3
_BLE_FLAG_WRITE = 0x0008


def _write_pwm_duty(pwm, duty: int):
    duty = max(0, min(1023, int(duty)))
    duty_u16 = getattr(pwm, "duty_u16", None)
    if duty_u16 is not None:
        duty_u16(duty * 65535 // 1023)
    else:
        pwm.duty(duty)

#servo đây
class BLEControlServer:
    def __init__(self):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.config(gap_name=BLE_NAME)
        self.ble.irq(self._irq)
        services = (
            (
                bluetooth.UUID(0xFFF0),
                ((bluetooth.UUID(0xFFF1), _BLE_FLAG_WRITE),),
            ),
        )
        handles = self.ble.gatts_register_services(services)
        self.value_handle = handles[0][0]
        self.pending_packet = None
        self.connected = False
        self._advertise()

    def _advertise(self):
        name = BLE_NAME.encode()
        adv_data = bytes((2, 0x01, 0x06, len(name) + 1, 0x09)) + name
        self.ble.gap_advertise(100000, adv_data=adv_data)

    def _irq(self, event, data):
        if event == _BLE_IRQ_CENTRAL_CONNECT:
            self.connected = True
        elif event == _BLE_IRQ_CENTRAL_DISCONNECT:
            self.connected = False
            self.pending_packet = None
            self._advertise()
        elif event == _BLE_IRQ_GATTS_WRITE:
            _, value_handle = data
            if value_handle == self.value_handle:
                self.pending_packet = bytes(self.ble.gatts_read(value_handle))

    def take_packet(self):
        packet = self.pending_packet
        self.pending_packet = None
        return packet


class ServoController:
    """Điều khiển Servo chuẩn 50Hz trên ESP32 bằng xung PWM."""

    def __init__(
        self,
        pin_num: int,
        min_angle: int = 0,
        max_angle: int = 180,
        min_pulse_us: int = 500,
        max_pulse_us: int = 2500,
    ):
        if max_angle <= min_angle:
            raise ValueError("max_angle phải lớn hơn min_angle")
        if max_pulse_us <= min_pulse_us:
            raise ValueError("max_pulse_us phải lớn hơn min_pulse_us")

        self.pwm = PWM(Pin(pin_num), freq=50)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.current_angle = self._clamp_angle(90)
        self._write_angle(self.current_angle)

    def _angle_to_duty(self, angle: float) -> int:
        angle = self._clamp_angle(angle)
        pulse_us = self.min_pulse_us + (
            (angle - self.min_angle)
            * (self.max_pulse_us - self.min_pulse_us)
            / (self.max_angle - self.min_angle)
        )
        return int((pulse_us / 20000) * 1023)

    def _clamp_angle(self, angle: float) -> float:
        return max(self.min_angle, min(self.max_angle, angle))

    def _write_angle(self, angle: float):
        _write_pwm_duty(self.pwm, self._angle_to_duty(angle))

    def set_angle(self, angle: float, smooth_delay: float = 0.015):
        """Quay servo mượt về góc mong muốn."""
        if smooth_delay < 0:
            raise ValueError("smooth_delay không được âm")

        target = self._clamp_angle(angle)
        step = 2 if target >= self.current_angle else -2

        for pos in range(int(self.current_angle), int(target), step):
            self._write_angle(pos)
            if smooth_delay:
                time.sleep(smooth_delay)

        self._write_angle(target)
        self.current_angle = target


class DCMotor:
    """Điều khiển 1 động cơ DC qua 2 chân IN1, IN2 dùng PWM."""

    def __init__(self, in1_pin: int, in2_pin: int, freq: int = 1000):
        self.in1 = PWM(Pin(in1_pin), freq=freq)
        self.in2 = PWM(Pin(in2_pin), freq=freq)
        self.stop()

    def set_speed(self, speed: int):
        """speed nằm trong khoảng [-1023, 1023]: dương = tới, âm = lùi, 0 = dừng."""
        speed = max(-1023, min(1023, int(speed)))
        if speed > 0:
            _write_pwm_duty(self.in2, 0)
            _write_pwm_duty(self.in1, speed)
        elif speed < 0:
            _write_pwm_duty(self.in1, 0)
            _write_pwm_duty(self.in2, abs(speed))
        else:
            self.stop()

    def stop(self):
        _write_pwm_duty(self.in1, 0)
        _write_pwm_duty(self.in2, 0)


class NewmaniaV2:
    """Bộ điều khiển trung tâm cho robot bánh Mecanum Newmania v2."""
    #  điều chỉnh ở đây gồm lên /xuống và gắp/nhả :))
    MAX_SPEED = 1023
    SERVO_STEP_DEGREES = 2
    SERVO_STEP_DELAY = 0.015

    def __init__(self):
        # 4 bánh xe Mecanum: FL (Trước-Trái), FR (Trước-Phải), RL (Sau-Trái), RR (Sau-Phải)
        self.fl = DCMotor(16, 17)
        self.fr = DCMotor(18, 19)
        self.rl = DCMotor(25, 26)
        self.rr = DCMotor(27, 14)

        # 2 Servo: Servo 1 = Nâng/Hạ, Servo 2 = Gắp/Nhả
        self.lift_servo = ServoController(pin_num=13)
        self.grip_servo = ServoController(pin_num=15)

        # Đèn pin
        self.light = Pin(12, Pin.OUT)
        self.light.value(0)

        # Góc cấu hình cho cơ cấu nâng-hạ và gắp-nhả (tùy chỉnh theo cơ khí thực tế)
        self.ANGLE_LIFT = 135
        self.ANGLE_LOWER = 45
        self.ANGLE_GRIP = 110
        self.ANGLE_RELEASE = 35

        self.reset_pose()

    def _apply_wheels(self, fl: int, fr: int, rl: int, rr: int):
        self.fl.set_speed(fl)
        self.fr.set_speed(fr)
        self.rl.set_speed(rl)
        self.rr.set_speed(rr)

    # --- CÁC CHƯƠNG TRÌNH DI CHUYỂN ---
    def drive(
        self,
        forward: int = 0,
        strafe: int = 0,
        turn: int = 0,
        duration: float = None,
    ):
        """Phối hợp tiến/lùi, ngang và xoay; mỗi trục nhận giá trị -1023..1023."""
        if duration is not None and duration < 0:
            raise ValueError("duration không được âm")

        fl = forward - strafe - turn
        fr = forward + strafe + turn
        rl = forward + strafe - turn
        rr = forward - strafe + turn
        peak = max(abs(fl), abs(fr), abs(rl), abs(rr))
        if peak > self.MAX_SPEED:
            scale = self.MAX_SPEED / peak
            fl = int(fl * scale)
            fr = int(fr * scale)
            rl = int(rl * scale)
            rr = int(rr * scale)

        self._apply_wheels(fl, fr, rl, rr)
        if duration is not None:
            time.sleep(duration)
            self.stop()
# chương trình di chuyển
    def stop(self):
        """Dừng toàn bộ bánh xe."""
        self._apply_wheels(0, 0, 0, 0)

    def move_forward(self, speed: int = 850, duration: float = None):
        """Tiến thẳng."""
        self.drive(forward=abs(speed), duration=duration)

    def move_backward(self, speed: int = 850, duration: float = None):
        """Lùi thẳng."""
        self.drive(forward=-abs(speed), duration=duration)

    def strafe_left(self, speed: int = 850, duration: float = None):
        """Đi ngang sang trái (Mecanum)."""
        self.drive(strafe=abs(speed), duration=duration)

    def strafe_right(self, speed: int = 850, duration: float = None):
        """Đi ngang sang phải (Mecanum)."""
        self.drive(strafe=-abs(speed), duration=duration)

    def _set_servo_angles_together(self, movements):
        servos = []
        for servo, angle in movements:
            servos.append((servo, servo.current_angle, servo._clamp_angle(angle)))

        max_delta = max(abs(target - start) for _, start, target in servos)
        steps = int(max_delta / self.SERVO_STEP_DEGREES)
        if steps * self.SERVO_STEP_DEGREES < max_delta:
            steps += 1
        for step in range(1, steps + 1):
            for servo, start, target in servos:
                angle = start + (target - start) * step / steps
                servo._write_angle(angle)
            if step < steps:
                time.sleep(self.SERVO_STEP_DELAY)

        for servo, _, target in servos:
            servo.current_angle = target

    # --- CÁC CHƯƠNG TRÌNH SERVO (NÂNG/HẠ & GẮP/NHẢ) ---
    def lift(self):
        """Nâng cơ cấu lên."""
        self.lift_servo.set_angle(self.ANGLE_LIFT)

    def lower(self):
        """Hạ cơ cấu xuống."""
        self.lift_servo.set_angle(self.ANGLE_LOWER)

    def grip(self):
        """Gắp vật."""
        self.grip_servo.set_angle(self.ANGLE_GRIP)

    def release(self):
        """Nhả vật."""
        self.grip_servo.set_angle(self.ANGLE_RELEASE)

    def lift_and_grip(self):
        """Chuỗi thao tác: Gắp vật -> Nâng lên."""
        self.grip()
        time.sleep(0.2)
        self.lift()

    def lower_and_release(self):
        """Chuỗi thao tác: Hạ xuống -> Nhả vật."""
        self.lower()
        time.sleep(0.2)
        self.release()

    def reset_pose(self):
        """Đưa 2 servo về trạng thái sẵn sàng (hạ thấp và mở kẹp)."""
        self._set_servo_angles_together(
            (
                (self.lift_servo, self.ANGLE_LOWER),
                (self.grip_servo, self.ANGLE_RELEASE),
            )
        )

    def toggle_light(self):
        """Bật/tắt đèn pin."""
        self.light.value(not self.light.value())


def _apply_controller_buttons(robot, buttons, previous_buttons):
    pressed = buttons & ~previous_buttons

    if pressed & BUTTON_RESET:
        robot.reset_pose()
    elif pressed & BUTTON_PICK:
        robot.lift_and_grip()
    elif pressed & BUTTON_DROP:
        robot.lower_and_release()
    else:
        if pressed & BUTTON_LIFT:
            robot.lift()
        elif pressed & BUTTON_LOWER:
            robot.lower()

        if pressed & BUTTON_GRIP:
            robot.grip()
        elif pressed & BUTTON_RELEASE:
            robot.release()

    if pressed & BUTTON_LIGHT:
        robot.toggle_light()


def run_ble_robot():
    print("BLE robot ready; advertising as", BLE_NAME)
    robot = NewmaniaV2()
    try:
        receiver = BLEControlServer()
        previous_buttons = 0
        motion = (0, 0, 0)
        last_packet_ms = None
        was_connected = False

        while True:
            if receiver.connected != was_connected:
                if receiver.connected:
                    print("PS4 bridge connected")
                else:
                    print("PS4 bridge disconnected")
                was_connected = receiver.connected

            packet = receiver.take_packet()
            now = time.ticks_ms()

            if packet is not None:
                if len(packet) == PACKET_SIZE:
                    magic, forward, strafe, turn, buttons = struct.unpack(
                        PACKET_FORMAT, packet
                    )
                    if magic == PACKET_MAGIC:
                        forward = max(-1023, min(1023, forward))
                        strafe = max(-1023, min(1023, strafe))
                        turn = max(-1023, min(1023, turn))
                        motion = (forward, strafe, turn)
                        _apply_controller_buttons(
                            robot, buttons, previous_buttons
                        )
                        previous_buttons = buttons
                        last_packet_ms = now
                    else:
                        print("Ignoring invalid BLE packet magic")
                else:
                    print("Ignoring invalid BLE packet length")

            if (
                last_packet_ms is None
                or time.ticks_diff(now, last_packet_ms) > CONTROL_TIMEOUT_MS
            ):
                motion = (0, 0, 0)
                previous_buttons = 0
                last_packet_ms = None
                robot.stop()
            else:
                robot.drive(*motion)

            time.sleep_ms(20)
    finally:
        robot.stop()
                                        

if __name__ == "__main__":
    if HARDWARE_AVAILABLE:
        run_ble_robot()
    else:
        if sys.implementation.name == "micropython":
            print(
                "Firmware hiện tại không phải ESP32. "
                "Hãy nạp MicroPython dành cho ESP32-S3 rồi chạy lại."
            )
        else:
            print(
                "Đây là firmware chạy trên ESP32, không chạy trực tiếp bằng Python "
                "trên máy tính. Hãy kết nối ESP32 bằng cáp USB dữ liệu và chạy "
                "tác vụ VS Code 'Robot: upload and run (MicroPython)'."
            )

            