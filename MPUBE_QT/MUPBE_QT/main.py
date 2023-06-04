import sys
from mpube_qt import Ui_Form
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMessageBox

task2_str = ""


class Pyqt5_Serial(QWidget, Ui_Form):
    def __init__(self):
        super(Pyqt5_Serial, self).__init__()
        self.setupUi(self)
        self.init()
        self.setWindowTitle("MPUBE_QT")
        self.ser = serial.Serial()
        self.port_check()

        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.data_num_sended = 0

    def init(self):
        # 串口检测按钮
        self.pushButton_2.clicked.connect(self.port_check)

        # 打开串口按钮
        self.pushButton.clicked.connect(self.port_open)

        # 关闭串口按钮
        self.pushButton_3.clicked.connect(self.port_close)

        # 定时器接收数据
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.data_receive)

    # 串口检测
    def port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.comboBox.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.comboBox.addItem(port[0])

    # 打开串口
    def port_open(self):
        self.ser.port = self.comboBox.currentText()
        self.ser.baudrate = 9600
        self.ser.bytesize = 8
        self.ser.stopbits = 1
        self.ser.parity = serial.PARITY_NONE

        try:
            self.ser.open()
        except:
            QMessageBox.critical(self, "Port Error", "此串口不能被打开！")
            return None

        # 打开串口接收定时器，周期为2ms
        self.timer.start(2)

        if self.ser.isOpen():
            self.pushButton.setEnabled(False)
            self.pushButton_3.setEnabled(True)

    # 关闭串口
    def port_close(self):
        self.timer.stop()
        try:
            self.ser.close()
        except:
            pass
        self.pushButton.setEnabled(True)
        self.pushButton_3.setEnabled(False)

    # 接收数据
    # TODO: 数据包一旦对不齐数据会闪退问题未考虑
    def data_receive(self):
        try:
            num = self.ser.inWaiting()
        except:
            self.port_close()
            return None
        if num > 0:
            data = self.ser.read(num)
            num = len(data)
            print(data)
            print(num)
            data_index = 0
            while (num):
                if data[data_index + 0] == 0x55:
                    id = data[data_index + 1]
                    if id == 0x00:
                        if data[data_index + 3] == 0x56:
                            if data[data_index + 2] == 0x00:
                                self.work_status_label.setText("OFF")
                            else:
                                self.work_status_label.setText("ON")

                            num = num - 4
                            data_index = data_index + 4
                        else:
                            num = 0
                            print("E:数据包错误")
                            break
                    elif id == 0x01:
                        if data[data_index + 5] == 0x56:
                            alfa_angle = 0.0
                            if data[data_index + 2] > 0:
                                alfa_angle += float(data[data_index + 3])
                                alfa_angle += float(data[data_index + 4]) / 100.0
                                self.alfa_label.setText("{:.2f}°".format(alfa_angle))
                            else:
                                alfa_angle -= float(data[data_index + 3])
                                alfa_angle -= float(data[data_index + 4]) / 100.0
                                self.alfa_label.setText("{:.2f}°".format(alfa_angle))

                            num = num - 6
                            data_index = data_index + 6
                        else:
                            num = 0
                            print("E:数据包错误")
                            break
                    elif id == 0x02:
                        beta_angle = 0.0
                        if data[data_index + 5] == 0x56:
                            if data[data_index + 2] > 0:
                                beta_angle += float(data[data_index + 3])
                                beta_angle += float(data[data_index + 4]) / 100.0
                                self.beta_label.setText("{:.2f}°".format(beta_angle))
                            else:
                                beta_angle -= float(data[data_index + 3])
                                beta_angle -= float(data[data_index + 4]) / 100.0
                                self.beta_label.setText("{:.2f}°".format(beta_angle))
                            num = num - 6
                            data_index = data_index + 6
                        else:
                            num = 0
                            print("E:数据包错误")
                            break
                    elif id == 0x03:
                        if data[data_index + 3] == 0x56:
                            self.top_status_label.setText(chr(data[data_index + 2]))
                            num = num - 4
                            data_index = data_index + 4
                        else:
                            num = 0
                            print("E:数据包错误")
                            break
                    elif id == 0x04:
                        if data[data_index + 3] == 0x56:
                            global task2_str
                            if '{:02X}'.format(data[data_index + 2]) not in task2_str:
                                task2_str = task2_str + '{:02X}'.format(data[data_index + 2]) + '->'
                                self.task3_label.setText(task2_str)
                            else:
                                print("W:拓展任务二网格路径重复")
                            num = num - 4
                            data_index = data_index + 4
                        else:
                            num = 0
                            print("E:数据包错误")
                            break

                    elif id == 0x05:
                        if data[data_index + 3] == 0x56:
                            task3_str = '{:02X}'.format(data[data_index + 2])
                            self.task2_label.setText(task3_str)
                            num = num - 4
                            data_index = data_index + 4
                        else:
                            num = 0
                            print("E:数据包错误")
                            break
                    elif id == 0x06:
                        if data[data_index + 3] == 0x56:
                            task3_str = '{:02X}'.format(data[data_index + 2])
                            # if(task3_str == '04' and self.task2_label.text() == "--"):
                            #
                            #     self.task2_label.setText("10")
                            self.groupBox_4.setTitle(task3_str)
                            num = num - 4
                            data_index = data_index + 4
                        else:
                            num = 0
                            print("E:数据包错误")
                            break
                    else:
                        num = 0
                        print("E:数据包ID异常")
                        break

                else:
                    num = 0
                    print("E:数据包错误")
        else:
            pass


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myshow = Pyqt5_Serial()
    myshow.show()
    sys.exit(app.exec_())
