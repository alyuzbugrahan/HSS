import sys
import time
import cv2
from pyzbar import pyzbar
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

        self.model3 = YOLO('görev3_demo.pt')
        self.active_model = None

        # Kamera başta kapalı
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.kamera_guncelle)

        self.detecting = False
        self.target_classes = []
        self.qr_data = []

        # Butonlar
        self.ui.gorev3Button.clicked.connect(self.gorev3Button)
        self.ui.angajmankabulButton.clicked.connect(self.angajman_mod)

        # Tablo
        self.ui.tableWidget_results.setColumnCount(4)
        self.ui.tableWidget_results.setHorizontalHeaderLabels(["ID", "Class", "Confidence", "BBox"])

        # Sabit resim
        self.label_2 = QLabel(self)
        self.label_2.setGeometry(920, 550, 231, 211)
        self.label_2.setScaledContents(True)
        pixmap = QPixmap("sahi.png")
        self.label_2.setPixmap(pixmap)

    def start_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            QMessageBox.critical(self, "Kamera Hatası", "Kamera başlatılamadı.")
            return
        self.timer.start(30)

    def kamera_guncelle(self):
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        annotated_frame = frame.copy()

        if self.detecting and self.active_model:
            results = self.active_model.predict(source=frame, verbose=False)
            filtered_boxes = []

            for box in results[0].boxes:
                label = results[0].names[int(box.cls)]
                if label in self.target_classes:
                    filtered_boxes.append(box)

            if filtered_boxes:
                results[0].boxes = filtered_boxes
                annotated_frame = results[0].plot()
                self.update_results_table(results)
            else:
                self.ui.tableWidget_results.setRowCount(0)

        # QLabel'e aktar
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

    def gorev3Button(self):
        self.active_model = self.model3
        self.detecting = False
        QMessageBox.information(self, "Görev 3", "Model 3 yüklendi. Angajman butonunu kullanabilirsiniz.")

    def angajman_mod(self):
        def qr_and_class():
            token_sequence = ['qr', 'yolo']
            phase_index = 0

            detected_qr_data = []
            detected_yolo_data = []
            yolo_detected = False

            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                QMessageBox.critical(self, "Hata", "QR/YOLO penceresi için kamera açılamadı.")
                return

            print("[INFO] QR ve YOLO tespiti başlatıldı. 'q' ile çıkabilirsiniz.")

            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                current_phase = token_sequence[phase_index]

                if current_phase == 'qr':
                    barcodes = pyzbar.decode(frame)
                    for barcode in barcodes:
                        data = barcode.data.decode('utf-8')
                        if data not in detected_qr_data:
                            detected_qr_data.append(data)
                            print(f"[QR] {data}")
                        x, y, w, h = barcode.rect
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    if barcodes:
                        phase_index = token_sequence.index('yolo')

                elif current_phase == 'yolo' and not yolo_detected:
                    results = self.model3(frame)
                    for result in results:
                        for box in result.boxes:
                            conf = float(box.conf)
                            if conf >= 0.80:
                                cls = int(box.cls)
                                label = result.names[cls]
                                x1, y1, x2, y2 = map(int, box.xyxy[0])
                                detected_yolo_data.append({'label': label, 'confidence': conf, 'bbox': (x1, y1, x2, y2)})
                                print(f"[YOLO] {label} (%{conf*100:.1f}) at {x1,y1,x2,y2}")
                                yolo_detected = True
                                break
                        if yolo_detected:
                            break
                    if yolo_detected:
                        break

                cv2.putText(frame, f"Asama: {'QR' if current_phase == 'qr' else 'YOLO'}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow("QR ve YOLO Tespiti", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()

            # Gecikme ekle (kamera kapanması için)
            time.sleep(1)

            # Hedefleri kaydet
            self.qr_data = detected_qr_data
            self.target_classes = [d['label'] for d in detected_yolo_data]

            print("\n=== Özet ===")
            print("QR Kodlar:")
            for qr in detected_qr_data:
                print(f" - {qr}")
            if self.target_classes:
                print("Hedef Sınıflar:")
                for cls in self.target_classes:
                    print(f" - {cls}")
            else:
                print("Yüksek güvenilirlikte hedef bulunamadı.")

            # Arayüz kamerasını başlat
            self.start_camera()
            self.detecting = True

        qr_and_class()

    def closeEvent(self, event):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Form()
    window.show()
    sys.exit(app.exec_())
