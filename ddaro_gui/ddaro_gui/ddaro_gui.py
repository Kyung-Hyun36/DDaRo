import sys
import os
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QDateTime, Qt, QLocale, QVariantAnimation, QEasingCurve
from PyQt5.QtGui import QFont

import rclpy
from std_msgs.msg import String
from ddaro_gui.qt_to_ros import ROSNode


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


class ChecklistItem(QWidget):
    def __init__(self, text, parent=None):
        super(ChecklistItem, self).__init__(parent)
        layout = QHBoxLayout()
        self.checkbox = QCheckBox(text)
        self.remove_button = QPushButton("Remove")
        layout.addWidget(self.checkbox)
        layout.addWidget(self.remove_button)
        self.setLayout(layout)
        self.remove_button.clicked.connect(self.remove_item)

    def remove_item(self):
        self.deleteLater()  # 위젯 삭제
        self.listWidget.sortItems()


class FollowWindow(QMainWindow, SetBackground):  
    def __init__(self):
        super(FollowWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/followWindow.ui", self)
        self.listWidget.setSelectionMode(QListWidget.NoSelection)
        self.listWidget.setStyleSheet("QListWidget::item { outline: none; }")
        
        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setGeometry(850, 30, 250, 100)
        
        # 버튼 기능 설정
        self.changeButton.clicked.connect(self.goToGuideWindow)
        self.exitButton.clicked.connect(self.openExitWindow)
        self.addButton.clicked.connect(self.add_item)

    def goToGuideWindow(self):
        widget.setCurrentIndex(3)

    def openExitWindow(self):
        exitWindow.show()

    def add_item(self):
        # 텍스트 입력 필드에서 텍스트 가져오기
        text = self.textEdit.text()
        if text:
            # 커스텀 위젯 생성
            customWidget = ChecklistItem(text)
            # QListWidgetItem에 커스텀 위젯 추가
            item = QListWidgetItem()
            item.setSizeHint(customWidget.sizeHint())
            self.listWidget.addItem(item)
            self.listWidget.setItemWidget(item, customWidget)
            # 텍스트 입력 필드 지우기
            self.textEdit.clear()


class GuideWindow(QMainWindow, SetBackground):  
    def __init__(self):
        super(GuideWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/guideWindow_1f.ui", self)

        # init ros
        rclpy.init()
        self.ros_node = ROSNode()
        self.ros_node.start()
        
        # 현재 날짜와 시간을 표시할 라벨 설정
        self.dateLabel = DateTimeLabel(self)
        self.dateLabel.setGeometry(850, 30, 250, 100)
        
        # 버튼 기능 설정
        self.changeButton.clicked.connect(self.goToFollowWindow)
        self.exitButton.clicked.connect(self.openExitWindow)

        # go waypoint button
        self.fish.clicked.connect(self.go_fish_btn)
        self.meat.clicked.connect(self.go_meat_btn)
        

    def goToFollowWindow(self):
        widget.setCurrentIndex(2)

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


# class HelpWindow(QMainWindow, SetBackground):  
#     def __init__(self):
#         super(HelpWindow, self).__init__()
#         loadUi("helpWindow.ui", self)
#         self.preIndex = 0
        
#         # 버튼 기능 설정
#         self.button.clicked.connect(self.goToPreWindow)

#     def setPreviousIndex(self, index):
#         self.preIndex = index

#     def goToPreWindow(self):
#         widget.setCurrentIndex(self.preIndex)


class ExitWindow(QDialog, SetBackground):  
    def __init__(self):
        super(ExitWindow, self).__init__()
        loadUi("/home/hyun/ros2_ws/src/ddaro/ddaro_gui/ui/exitWindow.ui", self)
        self.setWindowTitle("Exit")
        
        # 버튼 기능 설정
        self.yesButton.clicked.connect(self.goToStartWindow)
        self.noButton.clicked.connect(self.closeDialog)
    
    def goToStartWindow(self):
        self.close()
        widget.setCurrentIndex(0)

    def closeDialog(self):
        self.close()


def main(args=None):
    global widget, exitWindow

    app = QApplication(sys.argv)

    widget = QtWidgets.QStackedWidget()
    startWindow = StartWindow()
    cameraWindow = CameraWindow()
    followWindow = FollowWindow()
    guideWindow = GuideWindow()
    # helpWindow = HelpWindow()
    exitWindow = ExitWindow()
    widget.addWidget(startWindow)
    widget.addWidget(cameraWindow)
    widget.addWidget(followWindow)
    widget.addWidget(guideWindow)
    # widget.addWidget(helpWindow)
    widget.showFullScreen()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
    