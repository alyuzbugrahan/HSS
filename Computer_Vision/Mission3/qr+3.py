import cv2
from pyzbar import pyzbar
from ultralytics import YOLO
import time

def qr_and_class():
    mode_order = 'qr_then_yolo'  # Change to 'yolo_then_qr' to run YOLO first

    token_sequence = mode_order.split('_then_')  # ['qr','yolo'] or ['yolo','qr']
    phase_index = 0  # start with first in sequence

    detected_qr_data = []     # storage for QR results
    detected_yolo_data = []   # storage for YOLO results
    yolo_detected = False     # flag to stop after first YOLO detection

    # Initialize video capture
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    # Load YOLOv8 model
    yolo_model = YOLO('görev3_demo.pt')  # replace with your model path
    confidence_threshold = 0.80  # only predictions above 80%

    print(f"Starting capture in '{mode_order}' mode. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_phase = token_sequence[phase_index]

        if current_phase == 'qr':
            # QR Code Detection
            barcodes = pyzbar.decode(frame)
            for barcode in barcodes:
                data = barcode.data.decode('utf-8')
                if data not in detected_qr_data:
                    detected_qr_data.append(data)
                    print(f"[QR] Detected: {data}")
                x, y, w, h = barcode.rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Move to YOLO phase if any QR found
            if barcodes:
                phase_index = token_sequence.index('yolo')

        elif current_phase == 'yolo' and not yolo_detected:
            # YOLOv8 Object Detection (stop after first detection)
            results = yolo_model(frame)
            for result in results:
                for box in result.boxes:
                    conf = float(box.conf)
                    if conf >= confidence_threshold:
                        cls = int(box.cls)
                        label = result.names[cls]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        detected_yolo_data.append({'label': label, 'confidence': conf, 'bbox': (x1, y1, x2, y2)})
                        print(f"[YOLO] Detected: {label} with confidence {conf:.2f} at {(x1, y1, x2, y2)}")
                        yolo_detected = True
                        break
                if yolo_detected:
                    break
            # Once YOLO detects something, stop inference
            if yolo_detected:
                break

    # Show frame with phase label
    cv2.putText(frame, f"Phase: {'QR' if current_phase=='qr' else 'YOLO'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
    cv2.imshow('QR & YOLOv11 Detection', frame)
    

    cap.release()
    cv2.destroyAllWindows()

    # Print summary of detections
    print("\n=== Detection Summary ===")
    print(f"Unique QR codes detected ({len(detected_qr_data)}):")
    for qr in detected_qr_data:
        print(f" - {qr}")

    if detected_yolo_data:
        print(f"YOLO first detection above {int(confidence_threshold*100)}% confidence:")
        det = detected_yolo_data[0]
        print(f" - {det['label']} (conf={det['confidence']:.2f}) at {det['bbox']}")
    else:
        print("No YOLO detections above threshold.")
