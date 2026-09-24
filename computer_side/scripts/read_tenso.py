import serial

tenso = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

if not tenso.is_open:
    print("Error")
    raise SystemError

while 1:
    try:
        line = tenso.readline().decode('utf-8').strip()
        print(line)
    except KeyboardInterrupt:
        break
    except UnicodeDecodeError:
        pass
