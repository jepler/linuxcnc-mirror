import sys
import os
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import Qt
import popplerqt5

def pdf_view(filename):
    """Return a Scrollarea showing the pages of the specified PDF file."""
    filename = os.path.expanduser(filename)
    if not os.path.exists(filename):
        print('No path:',filename)

    doc = popplerqt5.Poppler.Document.load(filename)
    doc.setRenderHint(popplerqt5.Poppler.Document.Antialiasing)
    doc.setRenderHint(popplerqt5.Poppler.Document.TextAntialiasing)

    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    widget = QtWidgets.QWidget()
    vbox = QtWidgets.QVBoxLayout()

    for i in range(0,doc.numPages()):
        label = QtWidgets.QLabel()
        label.setScaledContents(True)

        page = doc.page(i)
        image = page.renderToImage()

        label.setPixmap(QtGui.QPixmap.fromImage(image))
        vbox.addWidget(label)
    widget.setLayout(vbox)
    area.setWidget(widget)
    return area


def main():
    app = QtWidgets.QApplication(sys.argv)
    argv = QtWidgets.QApplication.arguments()
    if len(argv) < 2:
        filename = '~/emc/nc_files/3D_Chips.pdf'
    else:
        filename = argv[-1]
    view = pdf_view(filename)
    view.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()