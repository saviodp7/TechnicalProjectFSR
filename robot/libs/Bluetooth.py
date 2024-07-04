from machine import UART, Pin
import time

class Bluetooth:
    
    def __init__(self, id=0, baudrate=0, rx=0, tx=0):
        self._id = id
        self._rx = rx
        self._tx = tx
        self._baudrate = baudrate
        if self._baudrate and (self._rx or self._tx): 
            self._bluetooth = UART(self._id, rx=self._rx, tx=self._tx, baudrate=self._baudrate)
        elif self._rx or self._tx: 
            self._bluetooth = UART(self._id, rx=self._rx, tx=self._tx)
        elif self._baudrate:
            self._bluetooth = UART(self._id, self._baudrate)
        else: 
            self._bluetooth = UART(self._id, 9600)
        time.sleep(0.1)
            
    def read(self) -> list[str] | None:
        """Read the last csv string sended through BT as a list"""
        if self._bluetooth.any():
            data = self._bluetooth.read()
            data = data.decode()
            data_list = data.split()[-1].split(",")
            return data_list
        else:
            return None
        
    def write(self, *args):
        data_csv = ",".join([f"{num:.{5}f}" for num in args])
        message = data_csv+"\n"
        self._bluetooth.write(message.encode())
        return message
        
if __name__ == "__main__":
    bluetooth = Bluetooth()
    prev_time = time.time_ns()
    i = 0
    while True:
        now = time.time_ns()
        if now - prev_time > 0.5e+9:
            prev_time = now
            data = bluetooth.read()
            print(f'[{time.time_ns()}] : {data}')
            msg = bluetooth.write(i,i+1,i+2)
            i += 1
             