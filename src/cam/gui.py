import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QUrl
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget

class VideoPlayer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GStreamer Player")
        self.setGeometry(100, 100, 640, 480)

        layout = QVBoxLayout()

        self.video_widget = QVideoWidget()
        layout.addWidget(self.video_widget)

        self.setLayout(layout)

        self.player = QMediaPlayer(None, QMediaPlayer.StreamPlayback)
        self.player.setVideoOutput(self.video_widget)

        media = QMediaContent(QUrl("udp://127.0.0.1:5000"))
        self.player.setMedia(media)
        self.player.play()

app = QApplication(sys.argv)
player = VideoPlayer()
player.show()
sys.exit(app.exec_())