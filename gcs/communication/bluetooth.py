import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QMessageBox
from PyQt5 import QtBluetooth


class BluetoothInterface(QWidget):

    def __init__(self, MAC_address="98:DA:50:02:38:17", parent=None):
        super(BluetoothInterface, self).__init__(parent)
        # "98:DA:50:02:14:09"

        # Initialize Bluetooth socket
        self.sock = QtBluetooth.QBluetoothSocket(QtBluetooth.QBluetoothServiceInfo.RfcommProtocol)
        self.sock.connected.connect(self.connectedToBluetooth)
        self.sock.readyRead.connect(self.receivedBluetoothMessage)
        self.sock.disconnected.connect(self.disconnectedFromBluetooth)
        self.sock.error.connect(self.socketError)
        port = 1
        self.sock.connectToService(QtBluetooth.QBluetoothAddress(MAC_address), port)

        self.last_message = ""

    def socketError(self, _error):
        error_message = self.sock.errorString()
        QMessageBox.critical(self, "Bluetooth Error", error_message)

    def connectedToBluetooth(self):
        print('Connected to Bluetooth')
        QMessageBox.information(self, "Bluetooth Connection", "Connected to Bluetooth device successfully.")

    def disconnectedFromBluetooth(self):
        print('Disconnected from Bluetooth')
        QMessageBox.warning(self, "Bluetooth Disconnection", "Disconnected from Bluetooth device.")

    def receivedBluetoothMessage(self):
        while self.sock.canReadLine():
            line = self.sock.readLine()
            self.last_message = (str(line, "utf-8")).split(",")

    def sendBluetoothMessage(self, message: str):
        self.sock.write(message.encode())


if __name__ == '__main__':
    # Deal with a Bluetooth bug on MAC
    if sys.platform == 'darwin':
        os.environ['QT_EVENT_DISPATCHER_CORE_FOUNDATION'] = '1'

    app = QApplication(sys.argv)
    bt_interface = BluetoothInterface()
    sys.exit(app.exec_())
