from PyQt5.QtWidgets import QApplication
# from manuel import Form
# from otonom import Form
from angajman2 import Form

uygulama = QApplication([])
pencere = Form()  # başlangıç formunuz hangisiyse onun adı verilir
pencere.show() 
uygulama.exec()