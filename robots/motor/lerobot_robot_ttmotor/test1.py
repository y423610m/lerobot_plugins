import lgpio
import time

def Test1():
    chip = 4
    line = 17
    
    h = lgpio.gpiochip_open(chip)
    lgpio.gpio_claim_output(h, line)
    
    for i in range(10000):
        print('1')
        lgpio.gpio_write(h, line, 1)
        time.sleep(1)
        print('0')
        lgpio.gpio_write(h, line, 0)
        time.sleep(1)
    
    print('done')

def Test2():
    chip = 4
    line_pwm = 18
    line_out1 = 27
    line_out2 = 22

    h = lgpio.gpiochip_open(chip)


    lgpio.tx_pwm(h, line_pwm, 10000, 10)
    lgpio.gpio_claim_output(h, line_out1)
    lgpio.gpio_claim_output(h, line_out2)
    lgpio.gpio_write(h, line_out2, 0)

    for i in range(10000):
        print('1')
        lgpio.gpio_write(h, line_out1, 1)
        lgpio.gpio_write(h, line_out2, 0)
        time.sleep(1)
        print('0')
        lgpio.gpio_write(h, line_out1, 0)
        lgpio.gpio_write(h, line_out2, 1)
        time.sleep(1)

def Test3():
    h = lgpio.gpiochip_open(4)

    IN1 = 27
    IN2 = 22
    PWM = 17

    # Configure direction pins
    lgpio.gpio_claim_output(h, IN1)
    lgpio.gpio_claim_output(h, IN2)

    # Configure PWM pin (software PWM because GPIO17 has no hardware PWM)
    # Frequency 10 kHz is good for motors
    PWM_FREQ = 10000

    # Helper: set direction
    def forward():
        lgpio.gpio_write(h, IN1, 1)
        lgpio.gpio_write(h, IN2, 0)

    def backward():
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 1)

    def stop():
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 0)

    # Helper: set speed (0.0–1.0)
    def set_speed(duty):
        # Software PWM uses 0.0–1.0 duty range
        lgpio.tx_pwm(h, PWM, PWM_FREQ, duty)

    # Run forward
    forward()
    set_speed(0.5)   # 50% speed
    time.sleep(2)

    backward()
    set_speed(0.5)   # 50% speed
    time.sleep(2)

    set_speed(1.0)   # full speed
    time.sleep(2)

    # Stop motor
    stop()
    set_speed(0)

    lgpio.gpiochip_close(h)

def Test4():
    chip = 4
    line_pwm = 13

    h = lgpio.gpiochip_open(chip)

    lgpio.tx_pwm(h, line_pwm, 50, 0)

    for i in range(10000):
        print('1')
        lgpio.tx_pwm(h, line_pwm, 50, 0)
        time.sleep(1)

        print('0')
        lgpio.tx_pwm(h, line_pwm, 50, 1)
        time.sleep(1)

        print('0')
        lgpio.tx_pwm(h, line_pwm, 50, 0.5)
        time.sleep(1)


if __name__ == '__main__':
    # Test2()
    Test4()