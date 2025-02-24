import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set the window title
        self.setWindowTitle("Secure Localhost Viewer")

        # Create a QWebEngineView widget
        self.browser = QWebEngineView()
        
        # Set the URL to localhost with the secret key
        self.browser.setUrl(QUrl('http://localhost:52000/?53000'))  # Secure URL
        
        # Set the central widget of the window to the browser
        self.setCentralWidget(self.browser)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
