import sys
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QDateTime, Qt, QLocale, QVariantAnimation, QEasingCurve, QEvent
from PyQt5.QtGui import QFont

import rclpy
from std_msgs.msg import String
from ddaro_gui.qt_to_ros import ROSNode
from hangul_utils import join_jamos, split_syllables

class DateTimeLabel(QLabel):
    def __init__(self, parent=None):
        super(DateTimeLabel, self).__init__(parent)

        # 폰트 설정
        self.setStyleSheet('font: 51 25pt "Gmarket Sans TTF";')
        self.setAlignment(Qt.AlignCenter)  # 가운데 정렬
        
        # 타이머 설정 (1초마다 시계 업데이트)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateClock)
        self.timer.start(1000)  # 밀리초 단위로 설정
        
        # 최초 업데이트
        self.updateClock()

    def updateClock(self):
        # 현재 날짜와 시간 가져오기
        currentDateTime = QDateTime.currentDateTime()
        
        # 현재 로케일에서 한글 요일로 표시
        locale = QLocale(QLocale.Korean)
        formattedDateTime = currentDateTime.toString("yyyy.MM.dd(") + locale.toString(currentDateTime.date(), "ddd") + ")\n" + currentDateTime.toString("hh:mm:ss")

        # 라벨에 표시
        self.setText(formattedDateTime)


class SetBackground:
    # 배경색 하얀색으로 설정
    def __init__(self):
        self.setStyleSheet("background-color: white;")


class StartWindow(QMainWindow, SetBackground):
    def __init__(self):
        super(StartWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/startWindow.ui", self)

        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setStyleSheet('font: 41 30pt "Gmarket Sans TTF";')
        self.dateLabel.setAlignment(Qt.AlignRight)  # 우측 정렬
        self.dateLabel.setGeometry(1610, 30, 290, 120)

        # "startText" label에 애니메이션 효과 적용
        self.startText_animation = QVariantAnimation(self)
        self.startText_animation.setDuration(1000)  # 애니메이션 지속 시간 (밀리초)
        self.startText_animation.setStartValue(1.0)
        self.startText_animation.setEndValue(1.2)
        self.startText_animation.setEasingCurve(QEasingCurve.OutElastic)  # 애니메이션 이징 커브 설정
        self.startText_animation.setLoopCount(-1)  # 무한 반복
        self.startText_animation.valueChanged.connect(self.animateStartText)

        # 애니메이션 시작
        self.startText_animation.start()

    def animateStartText(self, value):
        font = self.startText.font()
        font.setPointSizeF(50 * value)  # 폰트 크기를 애니메이션 값에 따라 변경
        self.startText.setFont(font)
        
    def mousePressEvent(self, event):
        widget.setCurrentIndex(1)
        

class CameraWindow(QMainWindow, SetBackground):  
    def __init__(self):
        super(CameraWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/cameraWindow.ui", self)
        
        # 버튼 기능 설정
        self.backButton.clicked.connect(self.goToStartWindow)

    def goToStartWindow(self):
        widget.setCurrentIndex(0)

    def mousePressEvent(self, event):
        widget.setCurrentIndex(2)


class ParkWindow(QMainWindow, SetBackground):  
    def __init__(self):
        super(ParkWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/parkWindow.ui", self)

        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setStyleSheet('font: 41 28pt "Gmarket Sans TTF";')
        self.dateLabel.setAlignment(Qt.AlignRight)  # 우측 정렬
        self.dateLabel.setGeometry(1610, 20, 290, 80)

        # 버튼 기능 설정
        for btn in self.findChildren(QPushButton):
            btn.clicked.connect(self.openMsgWindow)

    def openMsgWindow(self):
        global parkPose

        button = self.sender()
        parkPose = button.text()
        msgWindow.show()


class MsgWindow(QDialog, SetBackground):  
    def __init__(self):
        super(MsgWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/msgWindow.ui", self)
        self.setWindowTitle("Check")
        self.setFixedSize(self.size())
        
        # 버튼 기능 설정
        self.yesButton.clicked.connect(self.goToFollowWindow)
        self.noButton.clicked.connect(self.closeDialog)
    
    def showEvent(self, event):
        self.parkLabel.setText(parkPose)

    def goToFollowWindow(self):
        self.close()
        widget.setCurrentIndex(3)

    def closeDialog(self):
        self.close()


class KeyboardDialog(QDialog):
    def __init__(self, lineEdit, parent=None):
        super().__init__(parent)
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/keyboardWindow.ui", self)
        self.setWindowTitle("Keyboard")
        self.setFixedSize(1270, 440)
        self.lineEdit = lineEdit

        # 각 버튼에 클릭 이벤트를 연결합니다.
        # self.pushButton.clicked.connect(self.button_clicked)
        for btn in self.findChildren(QPushButton):
            btn.clicked.connect(self.addText)

        self.spaceBtn.clicked.disconnect(self.addText)
        self.spaceBtn.clicked.connect(self.addSpace)

        self.backspaceBtn.clicked.disconnect(self.addText)
        self.backspaceBtn.clicked.connect(self.backSpace)

        self.enterBtn.clicked.disconnect(self.addText)
        self.enterBtn.clicked.connect(self.enter)

    def showEvent(self, event):
        self.textBox.setText(self.lineEdit.text())

    def addText(self):
        button = self.sender()
        if button:
            text = button.text()
            # 현재 textBox 텍스트를 자음과 모음으로 분리
            jamo_list = split_syllables(self.textBox.toPlainText())
            # 분리된 자음과 모음을 버튼의 텍스트를 조합하여 새로운 글자를 생성
            new_text = join_jamos(jamo_list + text)
            # 생성된 글자를 textBox 설정
            self.textBox.setText(new_text)
    
    def addSpace(self):
        # 기존 텍스트에 공백 추가
        new_text = join_jamos(self.textBox.toPlainText() + " ")
        # 생성된 글자를 textBox 설정
        self.textBox.setText(new_text)

    def backSpace(self):
        # 현재 textBox 텍스트를 자음과 모음으로 분리
        jamo_list = split_syllables(self.textBox.toPlainText())
        # 생성된 글자의 마지막 글자 제거
        new_text = join_jamos(jamo_list[:-1])
        self.textBox.setText(new_text)

    def enter(self):
        # textBox에 적힌 text 가져오기
        text = self.textBox.toPlainText()
        # textBox에 적힌 text를 lineEdit 설정
        self.lineEdit.setText(text)
        self.textBox.clear()
        self.close()


class FollowWindow(QMainWindow, SetBackground):  
    def __init__(self):
        super(FollowWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/followWindow.ui", self)
        
        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setGeometry(850, 30, 250, 100)

        # 텍스트 입력 필드를 터치할 때 키보드 표시
        self.textEdit.installEventFilter(self)  # 이벤트 필터 추가
        self.keyboard_dialog = KeyboardDialog(self.textEdit, self)

        # 버튼 기능 설정
        self.changeButton.clicked.connect(self.goToGuideWindow)
        self.exitButton.clicked.connect(self.openExitWindow)
        self.addButton.clicked.connect(self.add_item)

    def eventFilter(self, obj, event):
        if obj == self.textEdit and event.type() == QEvent.MouseButtonPress:
            self.showKeyboard()
        return super().eventFilter(obj, event)
    
    def showKeyboard(self):
        self.keyboard_dialog.exec_()

    def goToGuideWindow(self):
        widget.setCurrentIndex(guideFloor)

    def openExitWindow(self):
        exitWindow.show()

    def add_item(self):
        # 텍스트 입력 필드에서 텍스트 가져오기
        text = self.textEdit.text()
        if text:
            listFont = QFont("Gmarket Sans TTF", 20)

            # 커스텀 위젯 생성
            layout = QHBoxLayout()

            checkbox = QCheckBox(text)
            checkbox.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")  # 체크박스 크기 조정
            checkbox.setFont(listFont)

            removeButton = QPushButton("X")
            removeButton.setStyleSheet("background-color: none; border: none; font-size: 20px;")  # 배경과 테두리 없애기
            removeButton.setFixedSize(30, 30)  # 크기 조절


            layout.addWidget(checkbox)
            layout.addWidget(removeButton)

            customWidget = QWidget()
            customWidget.setLayout(layout)

            # QListWidgetItem에 커스텀 위젯 추가
            item = QListWidgetItem()
            item.setSizeHint(customWidget.sizeHint())
            self.listWidget.addItem(item)
            self.listWidget.setItemWidget(item, customWidget)

            # "Remove" 버튼에 항목 제거 기능 연결
            removeButton.clicked.connect(lambda _, item=item: self.remove_item(item))

            # 텍스트 입력 필드 지우기
            self.textEdit.clear()

    def remove_item(self, item):
        row = self.listWidget.row(item)
        self.listWidget.takeItem(row)


class GuideWindow1F(QMainWindow, SetBackground):  
    def __init__(self):
        super(GuideWindow1F, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/guideWindow_1f.ui", self)
        
        # init ros
        self.ros_node = ROSNode()
        self.ros_node.start()
        
        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setGeometry(850, 30, 250, 100)
        
        # 버튼 기능 설정
        self.changeButton.clicked.connect(self.goToFollowWindow)
        self.exitButton.clicked.connect(self.openExitWindow)
        self.floorButton.clicked.connect(self.changeFloor)

        # # go waypoint button
        # self.fish.clicked.connect(self.go_fish_btn)
        # self.meat.clicked.connect(self.go_meat_btn)
    
    def showEvent(self, event):
        global guideFloor
        guideFloor = 4


    def goToFollowWindow(self):
        widget.setCurrentIndex(3)

    def changeFloor(self):
        widget.setCurrentIndex(5)

    def openExitWindow(self):
        exitWindow.show()

    # waypoint button
    # ====================================================== #
    def nav_cammnd(self, msg):
        self.nav_msg = String()
        self.nav_msg.data = msg
        self.ros_node.pub_navigator.publish(self.nav_msg)

    def go_fish_btn(self):
        self.nav_cammnd('go_to_fish')
        self.statusBar().showMessage('go fish')

    def go_meat_btn(self):
        self.nav_cammnd('go_to_meat')
        self.statusBar().showMessage('go meat')
    # ====================================================== #
    
    def closeEvent(self, event):
        rclpy.shutdown()


class GuideWindowB1(QMainWindow, SetBackground):  
    def __init__(self):
        super(GuideWindowB1, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/guideWindow_b1.ui", self)

        # init ros
        self.ros_node = ROSNode()
        self.ros_node.start()
        
        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setGeometry(850, 30, 250, 100)
        
        # 버튼 기능 설정
        self.changeButton.clicked.connect(self.goToFollowWindow)
        self.exitButton.clicked.connect(self.openExitWindow)
        self.floorButton.clicked.connect(self.changeFloor)

        # # go waypoint button
        # self.fish.clicked.connect(self.go_fish_btn)
        # self.meat.clicked.connect(self.go_meat_btn)
    
    def showEvent(self, event):
        global guideFloor
        guideFloor = 5

        self.parkLabel.setText(parkPose)

    def goToFollowWindow(self):
        widget.setCurrentIndex(3)

    def changeFloor(self):
        widget.setCurrentIndex(4)

    def openExitWindow(self):
        exitWindow.show()

    # waypoint button
    # ====================================================== #
    def nav_cammnd(self, msg):
        self.nav_msg = String()
        self.nav_msg.data = msg
        self.ros_node.pub_navigator.publish(self.nav_msg)

    def go_fish_btn(self):
        self.nav_cammnd('go_to_fish')
        self.statusBar().showMessage('go fish')

    def go_meat_btn(self):
        self.nav_cammnd('go_to_meat')
        self.statusBar().showMessage('go meat')
    # ====================================================== #
    
    def closeEvent(self, event):
        rclpy.shutdown()


class ExitWindow(QDialog, SetBackground):  
    def __init__(self):
        super(ExitWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/exitWindow.ui", self)
        self.setWindowTitle("Exit")
        self.setFixedSize(self.size())
        
        # 버튼 기능 설정
        self.yesButton.clicked.connect(self.goToStartWindow)
        self.noButton.clicked.connect(self.closeDialog)
    
    def goToStartWindow(self):
        self.close()
        widget.setCurrentIndex(0)

    def closeDialog(self):
        self.close()


def main(args=None):
    global widget, exitWindow, parkPose, msgWindow, guideFloor

    app = QApplication(sys.argv)

    parkPose = "None"
    guideFloor = 4

    rclpy.init()

    widget = QtWidgets.QStackedWidget()
    startWindow = StartWindow()
    cameraWindow = CameraWindow()
    parkWindow = ParkWindow()
    msgWindow = MsgWindow()
    followWindow = FollowWindow()
    guideWindow1F = GuideWindow1F()
    guideWindowB1 = GuideWindowB1()
    exitWindow = ExitWindow()
    widget.addWidget(startWindow)
    widget.addWidget(cameraWindow)
    widget.addWidget(parkWindow)
    widget.addWidget(followWindow)
    widget.addWidget(guideWindow1F)
    widget.addWidget(guideWindowB1)
    widget.showFullScreen()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
    