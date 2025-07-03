import sys
import cv2
from PyQt5.QtGui import QImage, QPixmap 
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QMessageBox, QTableWidgetItem
from arayuz import Ui_MainWindow
from ultralytics import YOLO

class Form(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


        
        # YOLO modelleri
        self.model1 = YOLO('v1_demo.pt')
        self.model2 = YOLO('v2_demo.pt')

        self.active_model = None
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.kamera_guncelle)
        self.timer.start(30)

        # Takip değişkenleri
        self.tracking = False
        self.tracker = None
        self.last_bbox = None
        self.current_frame = None
        self.detecting = False  # sadece hedeftespit yapıldığında true olacak

        # Buton bağlantıları
        self.ui.fireButton.clicked.connect(self.ates_et)
        self.ui.gorev1Button.clicked.connect(self.gorev1Button)
        self.ui.gorev2Button.clicked.connect(self.gorev2Button)
        self.ui.hedefyonelButton.clicked.connect(self.hedefe_yonel)
        self.ui.hedeftespitButton.clicked.connect(self.hedeftespitButton)
        self.ui.hedeftakipButton.clicked.connect(self.hedeftakipButton)

        # Sonuç tablosu
        self.ui.tableWidget_results.setColumnCount(4)
        self.ui.tableWidget_results.setHorizontalHeaderLabels(["ID", "Class", "Confidence", "BBox"])

        # Sağ alt köşe sabit görsel
        self.label_2 = QLabel(self)
        self.label_2.setGeometry(920, 550, 231, 211)
        self.label_2.setScaledContents(True)
        pixmap = QPixmap("sahi.png")
        self.label_2.setPixmap(pixmap)

    def kamera_guncelle(self):
        ret, frame = self.cap.read()
        if ret:
            self.current_frame = frame
            if self.detecting and self.active_model is not None:
                results = self.active_model.predict(source=frame, verbose=False)
                annotated_frame = results[0].plot()
                self.update_results_table(results)
            else:
                annotated_frame = frame
                self.ui.tableWidget_results.setRowCount(0)
            # Takip varsa
            if self.tracking and self.tracker is not None:
                annotated_frame = self.track_object(annotated_frame)
            
            # Görüntüyü QLabel'e gönder
            rgb_image = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg).scaled(
                self.ui.cameraLabel.width(), self.ui.cameraLabel.height()
            )
            self.ui.cameraLabel.setPixmap(pixmap)

    def update_results_table(self, results):
        self.ui.tableWidget_results.setRowCount(0)
        for i, det in enumerate(results[0].boxes):
            self.ui.tableWidget_results.insertRow(i)
            self.ui.tableWidget_results.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self.ui.tableWidget_results.setItem(i, 1, QTableWidgetItem(results[0].names[int(det.cls)]))
            self.ui.tableWidget_results.setItem(i, 2, QTableWidgetItem(f"{float(det.conf):.2f}"))
            self.ui.tableWidget_results.setItem(i, 3, QTableWidgetItem(str([int(x) for x in det.xyxy[0].tolist()])))

    def gorev1Button(self):
        self.active_model = self.model1
        self.detecting = False
        QMessageBox.information(self, "Görev 1", "Model 1 (Görev 1) aktif edildi.")

    def gorev2Button(self):
        self.active_model = self.model2
        self.detecting = False
        QMessageBox.information(self, "Görev 2", "Model 2 (Görev 2) aktif edildi.")

    def hedeftespitButton(self):
        if self.active_model is None:
            QMessageBox.warning(self, "Uyarı", "Lütfen önce bir görev seçin.")
            return
        self.detecting = True
        QMessageBox.information(self, "Hedef Tespit", "Hedef tespiti başlatıldı.")

    def hedeftakipButton(self):
        if self.active_model is None:
            QMessageBox.warning(self, "Uyarı", "Lütfen önce bir görev/model seçin.")
            return

        if self.current_frame is None:
            QMessageBox.warning(self, "Uyarı", "Kamera görüntüsü alınamadı.")
            return

        results = self.active_model.predict(source=self.current_frame, verbose=False)

        if len(results[0].boxes) == 0:
            QMessageBox.warning(self, "Hedef Bulunamadı", "Takip edilecek hedef algılanamadı.")
            return

        box = results[0].boxes[0].xyxy[0].tolist()
        x1, y1, x2, y2 = map(int, box)
        w, h = x2 - x1, y2 - y1

        self.tracker = cv2.TrackerKCF_create()
        self.tracker.init(self.current_frame, (x1, y1, w, h))
        self.last_bbox = (x1, y1, w, h)
        self.tracking = True
        self.detecting = False
        self.active_model = None
        # artık sadece takip edecek
        QMessageBox.information(self, "Takip Başlatıldı", "KCF ile takip aktif.")

    def track_object(self, frame):
        success, box = self.tracker.update(frame)
        if success:
            x, y, w, h = [int(v) for v in box]
            x_shape, y_shape = frame.shape[1], frame.shape[0]
            obj_center = [int(x + w / 2), int(y + h / 2)]
            center = (x_shape // 2, y_shape // 2)

            cv2.line(frame, center, obj_center, (0, 250, 250), 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Balon", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(frame, "TAKİP ALGORİTMASI AKTIF", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Vuruş alanı çizimi
            tx1, ty1 = int(x_shape * 0.25), int(y_shape * 0.10)
            tx2, ty2 = int(x_shape * 0.75), int(y_shape * 0.90)
            cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), (0, 255, 255), 2)

            if self.last_bbox is not None:
                last_x, last_y, last_w, last_h = self.last_bbox
                last_center = (last_x + last_w // 2, last_y + last_h // 2)
                new_center = (x + w // 2, y + h // 2)

                if abs(last_center[0] - new_center[0]) > 50 or abs(last_center[1] - new_center[1]) > 50:
                    # Eğer çok büyük bir kayma varsa, takip durdurulur ve yeniden başlatılır
                    self.tracking = False
                    self.last_bbox = None
        else:
            # Eğer takip başarısız olduysa, KCF tracker'ı sıfırlayıp yeniden başlatıyoruz.
            self.tracking = False
            self.last_bbox = None
            # Yeni hedef tespiti yaparak yeniden takip başlatabilirsiniz.
            self.detecting = True
            QMessageBox.information(self, "Takip Kayboldu", "Takip kayboldu, yeniden hedef tespiti yapılacak.")
            # Bu, hedef tespitini yeniden başlatacaktır, ama isterseniz başka bir mekanizma da kurabilirsiniz.

        return frame

    
    def hedefe_yonel(self):
        QMessageBox.information(self, "Kayıt Güncelleme", "Kayıt başarıyla güncellendi")
    def ates_et(self):
        QMessageBox.information(self, "Kayıt Güncelleme", "Kayıt başarıyla güncellendi")
 
    def closeEvent(self, event):
        self.cap.release()
        event.accept()

