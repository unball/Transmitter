import serial
from time import sleep
from random import randint

esp32 = serial.Serial('/dev/ttyUSB0', 115200)

class Twiddle():
    def __init__(self):
        self.serial = None

    def closeSerial(self):
        if self.serial is not None: self.serial.close()

    

def twiddle(k, dk, ksi=.3, target=None):
    if not target:
        target = run_pid_test(*k)

    for i, _ in enumerate(k):
        k[i] += dk[i]
        new_error = run_pid_test(*k)

        if new_error < target:
            target = new_error
            dk[i] *= 1 + ksi

        else:
            k[i] -= 2*dk[i]
            new_error = run_pid_test(*k)

            if new_error < target:
                target = new_error
                dk[i] *= 1 + ksi

            else:
                k[i] += dk[i]
                dk[i] *= 1 - ksi

    return k, dk, ksi, target

def run_pid_test(kp, ki, kd):
    print(f"<{0},{0},{0},{0},{3},{kp},{ki},{kd},{9},{0},{0},{0}>")
    esp32.write(f"<{0},{0},{0},{0},{3},{100*kp},{100*ki},{100*kd},{9},{0},{0},{0}>".encode())
    sleep(8)
    error = esp32.readline()
    error = error.decode()
    print(error)
    return abs(float(error))

params = [[-1.858962031647976, -0.16864975434220458, 0.16686768215869935], [0.538265, 0.049981750000000005, 0.049981750000000005]]

if __name__ == "__main__":
    while True:
        command = input("(1) run one\n"
                        "(2) run n\n"
                        "(3) close\n").strip()

        if command == "3":
            break

        n = 1
        if command == "2":
            n = int(input("\tn: ").strip())

        for _ in range(n):
            params = twiddle(*params)
            print("-------------------------------------------------------------------------------",
                "best (kp, ki, kd):", params[0],
                "\nbest error:", params[3],
                "\ncurrent (dks): ", params[1])


    esp32.close()
