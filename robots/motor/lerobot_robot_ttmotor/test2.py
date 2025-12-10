from time import sleep
from gpiozero import Servo, LED, DigitalOutputDevice
# from gpiozero.pins.pigpio import PiGPIOFactory

def test1():
    servo =  Servo(13, min_pulse_width=1.3/1000, max_pulse_width=1.7/1000)

    servo.max()
    sleep(1)

    servo.mid()
    sleep(1)

    servo.min()
    sleep(1)

# works for servo
def test2():
    servo = Servo(13)

    while 1:
        servo.value = -0.5
        sleep(1)
    
        servo.value = 0.0
        sleep(1)
    
        servo.value = 0.5
        sleep(1)

def test3():

    # factory = LGPIOFactory(chip=0)

    pin_pwm = 18
    pin_ch1 = 27
    pin_ch2 = 22
    
    servo_pwm =  Servo(pin_pwm, frame_width=20/1000, min_pulse_width=0.1*20/1000, max_pulse_width=0.99*20/1000)
    servo_pwm.value = 1
    servo_pwm.max()

    servo_ch1 =  DigitalOutputDevice(pin_ch1)
    servo_ch2 =  DigitalOutputDevice(pin_ch2)

    while 1:
        print(1)
        servo_ch1.on()
        servo_ch2.off()
        sleep(3)

        print(2)
        servo_ch1.off()
        servo_ch2.on()
        sleep(3)


if __name__ == '__main__':
    test3()
