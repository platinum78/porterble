from serial import Serial
import struct, time

def main():
    ser = Serial("/dev/ttyACM0", baudrate=57600)
    ser.flushInput()
    ser.flushOutput()

    time.sleep(3)

    require_byte = b'\x04\n'
    ser.write(require_byte)
    byte_string = ser.readline()
    while len(byte_string) != 16:
        print("Size unmatch!:", len(byte_string), byte_string)
        ser.write(require_byte)
        byte_string = ser.readline()

    count = 0
    while True:
        ser.write(require_byte)
        byte_string = ser.readline()
        len(byte_string)
        data = struct.unpack("HHHHHHHcc", byte_string)
        count += 1
        print(data, count)
        # time.sleep(0.001)

if __name__ == "__main__":
    main()