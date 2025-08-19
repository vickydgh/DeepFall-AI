from machine import Pin, PWM
import time
import _thread

buzzer_pwm = PWM(Pin(15))
buzzer_pwm.freq(2000)  # 2000Hz frequency

def beep_pwm(duration=0.5):
    buzzer_pwm.duty(512)  # 50% duty cycle
    time.sleep(duration)
    buzzer_pwm.duty(0)    # Turn off
    time.sleep(0.2)       # 0.2 second gap between beeps

def one_short_beep():
    beep_pwm(0.2)

def two_short_beeps():
    beep_pwm(0.2)
    time.sleep(0.2)
    beep_pwm(0.2)

def three_short_beeps():
    beep_pwm(0.2)
    time.sleep(0.2)
    beep_pwm(0.2)
    time.sleep(0.2)
    beep_pwm(0.2)

def continuous_buzzer(duration=30):
    buzzer_pwm.duty(512)  # Turn on buzzer
    time.sleep(duration)
    buzzer_pwm.duty(0)    # Turn off buzzer

def one_long_beep():
    buzzer_pwm.duty(512)
    time.sleep(1.0)
    buzzer_pwm.duty(0)

def one_long_beep_every_3_seconds():
    def beep_loop():
        while True:
            one_long_beep()
            time.sleep(2)  # 3 seconds total cycle (1s beep + 2s silence)
    _thread.start_new_thread(beep_loop, ())
