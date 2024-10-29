# import sys
# import paramiko
# import json
# import threading
# import queue
# from datetime import datetime
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.figure import Figure
# from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
#                            QTabWidget, QLabel, QPushButton, QLineEdit, QGroupBox, 
#                            QFormLayout, QDateEdit, QTableWidget, QHBoxLayout, 
#                            QTableWidgetItem, QMessageBox, QSpinBox)
# from PyQt5.QtCore import QTimer, QDate, pyqtSignal, QObject

# class MplCanvas(FigureCanvas):
#     def __init__(self, parent=None, width=5, height=4, dpi=100):
#         fig = Figure(figsize=(width, height), dpi=dpi)
#         self.axes = fig.add_subplot(111)
#         super().__init__(fig)
#         fig.tight_layout()

# class DataReceiver(QObject):
#     data_received = pyqtSignal(float)
#     error_occurred = pyqtSignal(str)

#     def __init__(self):
#         super().__init__()
#         self.data_queue = queue.Queue()
#         self.running = False

#     def start_receiving(self, ssh_client):
#         self.running = True
#         thread = threading.Thread(target=self._receive_data, args=(ssh_client,))
#         thread.daemon = True
#         thread.start()

#     def stop_receiving(self):
#         self.running = False

#     def _receive_data(self, ssh_client):
#         try:
#             command = "python3 /home/pi/read_pressure.py"
#             stdin, stdout, stderr = ssh_client.exec_command(command)
            
#             while self.running:
#                 line = stdout.readline()
#                 if not line:
#                     break
#                 try:
#                     value = float(line.strip())
#                     self.data_received.emit(value)
#                 except ValueError as e:
#                     self.error_occurred.emit(f"데이터 변환 오류: {str(e)}")
#         except Exception as e:
#             self.error_occurred.emit(f"데이터 수신 오류: {str(e)}")

# class NetworkChecker:
#     @staticmethod
#     def test_connection(hostname):
#         """네트워크 연결 테스트"""
#         import platform
#         import subprocess
#         try:
#             param = '-n' if platform.system().lower() == 'windows' else '-c'
#             command = ['ping', param, '1', hostname]
#             return subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
#         except:
#             return False

#     @staticmethod
#     def check_ssh_port(hostname, port):
#         """SSH 포트 연결 테스트"""
#         import socket
#         try:
#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.settimeout(5)
#             result = sock.connect_ex((hostname, port))
#             sock.close()
#             return result == 0
#         except:
#             return False

# class DeviceInfo:
#     def __init__(self):
#         self.hostname = ""
#         self.username = ""
#         self.password = ""
#         self.port = 22  # 기본 SSH 포트
#         self.is_connected = False
#         self.ssh_client = None
#         self.data_receiver = DataReceiver()
#         self.connection_timeout = 10

#     def connect(self):
#         try:
#             # 1. 네트워크 연결 확인
#             if not NetworkChecker.test_connection(self.hostname):
#                 raise Exception("네트워크 연결 실패: 장치에 ping을 보낼 수 없습니다.")

#             # 2. SSH 포트 확인
#             if not NetworkChecker.check_ssh_port(self.hostname, self.port):
#                 raise Exception(f"SSH 포트({self.port}) 연결 실패: SSH 서비스가 실행 중인지 확인해주세요.")

#             # 3. SSH 연결
#             self.ssh_client = paramiko.SSHClient()
#             self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
#             # 연결 시도
#             self.ssh_client.connect(
#                 self.hostname,
#                 port=self.port,
#                 username=self.username,
#                 password=self.password,
#                 timeout=self.connection_timeout,
#                 banner_timeout=self.connection_timeout,
#                 auth_timeout=self.connection_timeout
#             )

#             # 4. 압력 센서 읽기 스크립트 설치
#             self._install_pressure_script()
            
#             self.is_connected = True
#             return True

#         except Exception as e:
#             error_msg = str(e)
#             if "WinError 10060" in error_msg:
#                 error_msg = "연결 시간 초과: 다음을 확인해주세요:\n" + \
#                            "1. 장치 IP 주소가 올바른지\n" + \
#                            "2. 장치가 같은 네트워크에 있는지\n" + \
#                            "3. SSH가 활성화되어 있는지\n" + \
#                            "4. 방화벽이 SSH 연결을 차단하고 있지 않은지"
            
#             print(f"연결 실패: {error_msg}")
#             self.is_connected = False
#             return False

#     def _install_pressure_script(self):
#         """압력 센서 읽기 스크립트 설치"""
#         sftp = self.ssh_client.open_sftp()
#         try:
#             with sftp.file('/home/pi/read_pressure.py', 'w') as f:
#                 f.write('''
# import serial
# import time

# # 시리얼 포트 설정
# try:
#     ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# except:
#     try:
#         ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
#     except:
#         print("Error: 시리얼 포트를 열 수 없습니다.")
#         exit(1)

# while True:
#     try:
#         if ser.in_waiting:
#             line = ser.readline().decode('utf-8').strip()
#             try:
#                 value = float(line)
#                 print(value)
#             except ValueError:
#                 pass
#     except:
#         print("Error: 데이터 읽기 실패")
#     time.sleep(0.1)
# ''')
#         finally:
#             sftp.close()

#         # pyserial 설치
#         stdin, stdout, stderr = self.ssh_client.exec_command('pip3 install pyserial')
#         if stdout.channel.recv_exit_status() != 0:
#             raise Exception("pyserial 설치 실패")

#     def start_data_collection(self):
#         if self.is_connected:
#             self.data_receiver.start_receiving(self.ssh_client)

#     def stop_data_collection(self):
#         self.data_receiver.stop_receiving()

#     def disconnect(self):
#         self.stop_data_collection()
#         if self.ssh_client:
#             self.ssh_client.close()
#             self.is_connected = False

# class PostureMonitorApp(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.device = DeviceInfo()
#         self.pressure_data = []
#         self.times = []
#         self.initUI()
        
#         self.device.data_receiver.data_received.connect(self.handle_new_data)
#         self.device.data_receiver.error_occurred.connect(self.handle_error)
        
#         self.update_timer = QTimer()
#         self.update_timer.timeout.connect(self.update_graph)
#         self.update_timer.start(1000)  # 1초마다 업데이트

#     def handle_new_data(self, value):
#         current_time = len(self.pressure_data)
#         self.pressure_data.append(value)
#         self.times.append(current_time)
        
#         # 60초 데이터만 유지
#         if len(self.pressure_data) > 60:
#             self.pressure_data.pop(0)
#             self.times.pop(0)
            
#         self.update_posture_status(value)
#         self.log_posture_data(value)

#     def handle_error(self, error_message):
#         QMessageBox.warning(self, '오류', error_message)

#     def update_posture_status(self, value):
#         if value > 80:
#             status = '불량 (너무 기대어 앉음)'
#             color = 'red'
#         elif value < 20:
#             status = '불량 (너무 앞으로 숙임)'
#             color = 'red'
#         else:
#             status = '양호'
#             color = 'green'
#         self.posture_status_label.setText(f'현재 자세: {status}')
#         self.posture_status_label.setStyleSheet(f'color: {color}')

#     def log_posture_data(self, value):
#         current_time = datetime.now().strftime('%H:%M:%S')
#         status = '불량' if value > 80 or value < 20 else '양호'
        
#         row_position = self.stats_table.rowCount()
#         self.stats_table.insertRow(row_position)
        
#         self.stats_table.setItem(row_position, 0, QTableWidgetItem(current_time))
#         self.stats_table.setItem(row_position, 1, QTableWidgetItem(status))
#         self.stats_table.setItem(row_position, 2, QTableWidgetItem(f'{value:.1f}'))

#         # 상태에 따라 행 색상 변경
#         color = 'pink' if status == '불량' else 'lightgreen'
#         for col in range(3):
#             self.stats_table.item(row_position, col).setBackground(QColor(color))

#     def update_graph(self):
#         if self.device.is_connected and self.pressure_data:
#             self.canvas.axes.clear()
#             self.canvas.axes.plot(self.times, self.pressure_data, 'b-')
#             self.canvas.axes.set_xlim(max(0, len(self.pressure_data) - 60), max(60, len(self.pressure_data)))
#             self.canvas.axes.set_ylim(0, 100)
#             self.canvas.axes.set_xlabel('시간 (초)')
#             self.canvas.axes.set_ylabel('압력')
#             self.canvas.axes.grid(True)
#             self.canvas.draw()

#     def initUI(self):
#         self.setWindowTitle('자세 교정 모니터링 시스템')
#         self.setGeometry(100, 100, 1000, 800)

#         main_widget = QWidget()
#         self.setCentralWidget(main_widget)
#         layout = QVBoxLayout()
#         main_widget.setLayout(layout)

#         # 탭 위젯 생성
#         tabs = QTabWidget()
#         layout.addWidget(tabs)

#         # 장치 정보 탭
#         device_tab = QWidget()
#         device_layout = QVBoxLayout()
        
#         # 연결 상태 표시
#         status_group = QGroupBox('연결 상태')
#         status_layout = QVBoxLayout()
#         self.status_label = QLabel('연결 상태: 미연결')
#         status_layout.addWidget(self.status_label)
#         status_group.setLayout(status_layout)
#         device_layout.addWidget(status_group)

#         # 연결 설정
#         settings_group = QGroupBox('연결 설정')
#         settings_layout = QFormLayout()
        
#         self.hostname_input = QLineEdit(self.device.hostname)
#         self.username_input = QLineEdit(self.device.username)
#         self.password_input = QLineEdit(self.device.password)
#         self.password_input.setEchoMode(QLineEdit.Password)
#         self.port_input = QSpinBox()
#         self.port_input.setRange(1, 65535)
#         self.port_input.setValue(22)
        
#         settings_layout.addRow('IP 주소:', self.hostname_input)
#         settings_layout.addRow('사용자명:', self.username_input)
#         settings_layout.addRow('비밀번호:', self.password_input)
#         settings_layout.addRow('SSH 포트:', self.port_input)
        
#         settings_group.setLayout(settings_layout)
#         device_layout.addWidget(settings_group)

#         # 연결 버튼
#         buttons_layout = QHBoxLayout()
#         connect_button = QPushButton('연결')
#         connect_button.clicked.connect(self.connect_device)
#         disconnect_button = QPushButton('연결 해제')
#         disconnect_button.clicked.connect(self.disconnect_device)
        
#         buttons_layout.addWidget(connect_button)
#         buttons_layout.addWidget(disconnect_button)
#         device_layout.addLayout(buttons_layout)
        
#         device_tab.setLayout(device_layout)

#         # 자세 분석 탭
#         posture_tab = QWidget()
#         posture_layout = QVBoxLayout()

#         status_group = QGroupBox('현재 자세 상태')
#         status_layout = QVBoxLayout()
#         self.posture_status_label = QLabel('현재 자세: 분석 중...')
#         status_layout.addWidget(self.posture_status_label)
#         status_group.setLayout(status_layout)
#         posture_layout.addWidget(status_group)

#         graph_group = QGroupBox('실시간 압력 데이터')
#         graph_layout = QVBoxLayout()
        
#         self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
#         graph_layout.addWidget(self.canvas)
        
#         graph_group.setLayout(graph_layout)
#         posture_layout.addWidget(graph_group)
        
#         posture_tab.setLayout(posture_layout)

#         # 기록 및 통계 탭
#         stats_tab = QWidget()
#         stats_layout = QVBoxLayout()
        
#         date_group = QGroupBox('날짜 선택')
#         date_layout = QHBoxLayout()
#         date_selector = QDateEdit()
#         date_selector.setCalendarPopup(True)
#         date_selector.setDate(QDate.currentDate())
#         date_layout.addWidget(date_selector)
#         date_group.setLayout(date_layout)
#         stats_layout.addWidget(date_group)
        
#         self.stats_table = QTableWidget()
#         self.stats_table.setColumnCount(3)
#         self.stats_table.setHorizontalHeaderLabels(['시간', '자세 상태', '압력값'])
#         self.stats_table.horizontalHeader().setStretchLastSection(True)
#         self.stats_table.setAlternatingRowColors(True)
#         stats_layout.addWidget(self.stats_table)
        
#         # 통계 버튼
#         stats_buttons_layout = QHBoxLayout()
#         export_button = QPushButton('데이터 내보내기')
#         export_button.clicked.connect(self.export_data)
#         clear_button = QPushButton('기록 지우기')
#         clear_button.clicked.connect(self.clear_stats)
#         stats_buttons_layout.addWidget(export_button)
#         stats_buttons_layout.addWidget(clear_button)
#         stats_layout.addLayout(stats_buttons_layout)
        
#         stats_tab.setLayout(stats_layout)

#         # 탭 추가
#         tabs.addTab(device_tab, '장치 정보')
#         tabs.addTab(posture_tab, '자세 분석')
#         tabs.addTab(stats_tab, '기록 및 통계')

#         self.statusBar().showMessage('시스템 준비')
        
#         # 자동 연결 시도
#         QTimer.singleShot(1000, self.auto_connect)

#     def connect_device(self):
#         self.device.hostname = self.hostname_input.text()
#         self.device.username = self.username_input.text()
#         self.device.password = self.password_input.text()
#         self.device.port = self.port_input.value()

#         if not self.device.hostname or not self.device.username or not self.device.password:
#             QMessageBox.warning(self, '입력 오류', '모든 연결 정보를 입력해주세요.')
#             return

#         self.statusBar().showMessage('연결 시도 중...')
#         if self.device.connect():
#             self.status_label.setText('연결 상태: 연결됨')
#             self.status_label.setStyleSheet('color: green')
#             self.statusBar().showMessage('장치 연결 성공')
#             self.save_connection_settings()
#             self.device.start_data_collection()
#         else:
#             self.status_label.setText('연결 상태: 연결 실패')
#             self.status_label.setStyleSheet('color: red')
#             self.statusBar().showMessage('장치 연결 실패')

#     def disconnect_device(self):
#         self.device.disconnect()
#         self.status_label.setText('연결 상태: 미연결')
#         self.status_label.setStyleSheet('color: black')
#         self.statusBar().showMessage('장치 연결 해제됨')

#     def auto_connect(self):
#         try:
#             with open('connection_settings.json', 'r') as f:
#                 settings = json.load(f)
#                 self.device.hostname = settings.get('hostname', '')
#                 self.device.username = settings.get('username', '')
#                 self.device.password = settings.get('password', '')
#                 self.device.port = settings.get('port', 22)
                
#                 self.hostname_input.setText(self.device.hostname)
#                 self.username_input.setText(self.device.username)
#                 self.password_input.setText(self.device.password)
#                 self.port_input.setValue(self.device.port)
                
#                 if all([self.device.hostname, self.device.username, self.device.password]):
#                     self.connect_device()
#         except FileNotFoundError:
#             self.statusBar().showMessage('저장된 연결 설정 없음')
#         except json.JSONDecodeError:
#             self.statusBar().showMessage('연결 설정 파일 손상됨')

#     def save_connection_settings(self):
#         settings = {
#             'hostname': self.device.hostname,
#             'username': self.device.username,
#             'password': self.device.password,
#             'port': self.device.port
#         }
#         try:
#             with open('connection_settings.json', 'w') as f:
#                 json.dump(settings, f)
#         except Exception as e:
#             QMessageBox.warning(self, '저장 오류', f'설정 저장 실패: {str(e)}')

#     def export_data(self):
#         try:
#             filename = f'posture_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
#             with open(filename, 'w') as f:
#                 f.write('시간,자세 상태,압력값\n')
#                 for row in range(self.stats_table.rowCount()):
#                     time = self.stats_table.item(row, 0).text()
#                     status = self.stats_table.item(row, 1).text()
#                     pressure = self.stats_table.item(row, 2).text()
#                     f.write(f'{time},{status},{pressure}\n')
#             QMessageBox.information(self, '내보내기 성공', f'데이터가 {filename}에 저장되었습니다.')
#         except Exception as e:
#             QMessageBox.warning(self, '내보내기 실패', f'데이터 내보내기 실패: {str(e)}')

#     def clear_stats(self):
#         reply = QMessageBox.question(self, '기록 삭제', 
#                                    '모든 기록을 삭제하시겠습니까?',
#                                    QMessageBox.Yes | QMessageBox.No)
        
#         if reply == QMessageBox.Yes:
#             self.stats_table.setRowCount(0)
#             self.statusBar().showMessage('기록이 삭제되었습니다.')

#     def closeEvent(self, event):
#         reply = QMessageBox.question(self, '종료', 
#                                    '프로그램을 종료하시겠습니까?',
#                                    QMessageBox.Yes | QMessageBox.No)
        
#         if reply == QMessageBox.Yes:
#             self.device.disconnect()
#             event.accept()
#         else:
#             event.ignore()

# if __name__ == '__main__':
#     import sys
#     from PyQt5.QtWidgets import QApplication
#     from PyQt5.QtGui import QColor
    
#     app = QApplication(sys.argv)
#     ex = PostureMonitorApp()
#     ex.show()
#     sys.exit(app.exec_())


import sys
import json
import socket
import threading
import queue
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QTabWidget, QLabel, QPushButton, QLineEdit, QGroupBox, 
                           QFormLayout, QTableWidget, QHBoxLayout, 
                           QTableWidgetItem, QMessageBox, QSpinBox)
from PyQt5.QtCore import QTimer, QDate, pyqtSignal, QObject
from PyQt5.QtGui import QColor

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        fig.tight_layout()

class DataReceiver(QObject):
    data_received = pyqtSignal(float)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.socket = None
        self.running = False

    def connect(self, host, port):
        """서버에 연결"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((host, port))
            return True
        except Exception as e:
            self.error_occurred.emit(str(e))
            return False

    def start_receiving(self):
        """데이터 수신 시작"""
        if self.socket:
            self.running = True
            thread = threading.Thread(target=self._receive_data)
            thread.daemon = True
            thread.start()

    def stop_receiving(self):
        """데이터 수신 중지"""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

    def _receive_data(self):
        """데이터 수신 처리"""
        buffer = ""
        while self.running:
            try:
                data = self.socket.recv(1024).decode()
                if not data:
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        value = json.loads(line)['pressure']
                        self.data_received.emit(float(value))
                    except (json.JSONDecodeError, KeyError, ValueError) as e:
                        self.error_occurred.emit(f"데이터 변환 오류: {str(e)}")
            except Exception as e:
                self.error_occurred.emit(f"데이터 수신 오류: {str(e)}")
                break

class PostureMonitorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.data_receiver = DataReceiver()
        self.pressure_data = []
        self.times = []
        self.initUI()
        
        self.data_receiver.data_received.connect(self.handle_new_data)
        self.data_receiver.error_occurred.connect(self.handle_error)
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_graph)
        self.update_timer.start(1000)  # 1초마다 업데이트

    def handle_new_data(self, value):
        current_time = len(self.pressure_data)
        self.pressure_data.append(value)
        self.times.append(current_time)
        
        # 60초 데이터만 유지
        if len(self.pressure_data) > 60:
            self.pressure_data.pop(0)
            self.times.pop(0)
            
        self.update_posture_status(value)
        self.log_posture_data(value)

    def handle_error(self, error_message):
        QMessageBox.warning(self, '오류', error_message)

    def update_posture_status(self, value):
        if value > 80:
            status = '불량 (너무 기대어 앉음)'
            color = 'red'
        elif value < 20:
            status = '불량 (너무 앞으로 숙임)'
            color = 'red'
        else:
            status = '양호'
            color = 'green'
        self.posture_status_label.setText(f'현재 자세: {status}')
        self.posture_status_label.setStyleSheet(f'color: {color}')

    def log_posture_data(self, value):
        current_time = datetime.now().strftime('%H:%M:%S')
        status = '불량' if value > 80 or value < 20 else '양호'
        
        row_position = self.stats_table.rowCount()
        self.stats_table.insertRow(row_position)
        
        self.stats_table.setItem(row_position, 0, QTableWidgetItem(current_time))
        self.stats_table.setItem(row_position, 1, QTableWidgetItem(status))
        self.stats_table.setItem(row_position, 2, QTableWidgetItem(f'{value:.1f}'))

        # 상태에 따라 행 색상 변경
        color = 'pink' if status == '불량' else 'lightgreen'
        for col in range(3):
            self.stats_table.item(row_position, col).setBackground(QColor(color))

    def update_graph(self):
        if self.data_receiver.socket and self.pressure_data:
            self.canvas.axes.clear()
            self.canvas.axes.plot(self.times, self.pressure_data, 'b-')
            self.canvas.axes.set_xlim(max(0, len(self.pressure_data) - 60), max(60, len(self.pressure_data)))
            self.canvas.axes.set_ylim(0, 100)
            self.canvas.axes.set_xlabel('시간 (초)')
            self.canvas.axes.set_ylabel('압력')
            self.canvas.axes.grid(True)
            self.canvas.draw()

    def initUI(self):
        self.setWindowTitle('자세 교정 모니터링 시스템')
        self.setGeometry(100, 100, 1000, 800)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()
        main_widget.setLayout(layout)

        # 탭 위젯 생성
        tabs = QTabWidget()
        layout.addWidget(tabs)

        # 장치 정보 탭
        device_tab = QWidget()
        device_layout = QVBoxLayout()
        
        # 연결 상태 표시
        status_group = QGroupBox('연결 상태')
        status_layout = QVBoxLayout()
        self.status_label = QLabel('연결 상태: 미연결')
        status_layout.addWidget(self.status_label)
        status_group.setLayout(status_layout)
        device_layout.addWidget(status_group)

        # 연결 설정
        settings_group = QGroupBox('연결 설정')
        settings_layout = QFormLayout()
        
        self.hostname_input = QLineEdit()
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(5000)
        
        settings_layout.addRow('서버 주소:', self.hostname_input)
        settings_layout.addRow('포트:', self.port_input)
        
        settings_group.setLayout(settings_layout)
        device_layout.addWidget(settings_group)

        # 연결 버튼
        buttons_layout = QHBoxLayout()
        connect_button = QPushButton('연결')
        connect_button.clicked.connect(self.connect_device)
        disconnect_button = QPushButton('연결 해제')
        disconnect_button.clicked.connect(self.disconnect_device)
        
        buttons_layout.addWidget(connect_button)
        buttons_layout.addWidget(disconnect_button)
        device_layout.addLayout(buttons_layout)
        
        device_tab.setLayout(device_layout)

        # 자세 분석 탭
        posture_tab = QWidget()
        posture_layout = QVBoxLayout()

        status_group = QGroupBox('현재 자세 상태')
        status_layout = QVBoxLayout()
        self.posture_status_label = QLabel('현재 자세: 분석 중...')
        status_layout.addWidget(self.posture_status_label)
        status_group.setLayout(status_layout)
        posture_layout.addWidget(status_group)

        graph_group = QGroupBox('실시간 압력 데이터')
        graph_layout = QVBoxLayout()
        
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        graph_layout.addWidget(self.canvas)
        
        graph_group.setLayout(graph_layout)
        posture_layout.addWidget(graph_group)
        
        posture_tab.setLayout(posture_layout)

        # 기록 및 통계 탭
        stats_tab = QWidget()
        stats_layout = QVBoxLayout()
        
        self.stats_table = QTableWidget()
        self.stats_table.setColumnCount(3)
        self.stats_table.setHorizontalHeaderLabels(['시간', '자세 상태', '압력값'])
        self.stats_table.horizontalHeader().setStretchLastSection(True)
        self.stats_table.setAlternatingRowColors(True)
        stats_layout.addWidget(self.stats_table)
        
        # 통계 버튼
        stats_buttons_layout = QHBoxLayout()
        export_button = QPushButton('데이터 내보내기')
        export_button.clicked.connect(self.export_data)
        clear_button = QPushButton('기록 지우기')
        clear_button.clicked.connect(self.clear_stats)
        stats_buttons_layout.addWidget(export_button)
        stats_buttons_layout.addWidget(clear_button)
        stats_layout.addLayout(stats_buttons_layout)
        
        stats_tab.setLayout(stats_layout)

        # 탭 추가
        tabs.addTab(device_tab, '장치 정보')
        tabs.addTab(posture_tab, '자세 분석')
        tabs.addTab(stats_tab, '기록 및 통계')

    def connect_device(self):
        """서버에 연결"""
        host = self.hostname_input.text()
        port = self.port_input.value()

        if not host:
            QMessageBox.warning(self, '입력 오류', '서버 주소를 입력해주세요.')
            return

        self.statusBar().showMessage('연결 시도 중...')
        
        if self.data_receiver.connect(host, port):
            self.status_label.setText('연결 상태: 연결됨')
            self.status_label.setStyleSheet('color: green')
            self.statusBar().showMessage('서버 연결 성공')
            self.save_connection_settings()
            self.data_receiver.start_receiving()
        else:
            self.status_label.setText('연결 상태: 연결 실패')
            self.status_label.setStyleSheet('color: red')
            self.statusBar().showMessage('서버 연결 실패')

    def disconnect_device(self):
        """서버 연결 해제"""
        self.data_receiver.stop_receiving()
        self.status_label.setText('연결 상태: 미연결')
        self.status_label.setStyleSheet('color: black')
        self.statusBar().showMessage('서버 연결 해제됨')

    def auto_connect(self):
        """저장된 연결 설정으로 자동 연결"""
        try:
            with open('connection_settings.json', 'r') as f:
                settings = json.load(f)
                host = settings.get('host', '')
                port = settings.get('port', 5000)
                
                self.hostname_input.setText(host)
                self.port_input.setValue(port)
                
                if host:
                    self.connect_device()
        except FileNotFoundError:
            self.statusBar().showMessage('저장된 연결 설정 없음')
        except json.JSONDecodeError:
            self.statusBar().showMessage('연결 설정 파일 손상됨')

    def save_connection_settings(self):
        """연결 설정 저장"""
        settings = {
            'host': self.hostname_input.text(),
            'port': self.port_input.value()
        }
        try:
            with open('connection_settings.json', 'w') as f:
                json.dump(settings, f)
        except Exception as e:
            QMessageBox.warning(self, '저장 오류', f'설정 저장 실패: {str(e)}')

    def export_data(self):
        """측정 데이터 CSV 파일로 내보내기"""
        try:
            filename = f'posture_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
            with open(filename, 'w', encoding='utf-8') as f:
                f.write('시간,자세 상태,압력값\n')
                for row in range(self.stats_table.rowCount()):
                    time = self.stats_table.item(row, 0).text()
                    status = self.stats_table.item(row, 1).text()
                    pressure = self.stats_table.item(row, 2).text()
                    f.write(f'{time},{status},{pressure}\n')
            QMessageBox.information(self, '내보내기 성공', f'데이터가 {filename}에 저장되었습니다.')
        except Exception as e:
            QMessageBox.warning(self, '내보내기 실패', f'데이터 내보내기 실패: {str(e)}')

    def clear_stats(self):
        """측정 기록 삭제"""
        reply = QMessageBox.question(self, '기록 삭제', 
                                   '모든 기록을 삭제하시겠습니까?',
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.stats_table.setRowCount(0)
            self.statusBar().showMessage('기록이 삭제되었습니다.')

    def closeEvent(self, event):
        """프로그램 종료 처리"""
        reply = QMessageBox.question(self, '종료', 
                                   '프로그램을 종료하시겠습니까?',
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.data_receiver.stop_receiving()
            event.accept()
        else:
            event.ignore()

if __name__ == '__main__':
    import sys
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtGui import QColor
    
    app = QApplication(sys.argv)
    ex = PostureMonitorApp()
    ex.show()
    sys.exit(app.exec_())