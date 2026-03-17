import cv2
import mediapipe as mp
import numpy as np
import os
import pickle
import time
from collections import deque

from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline


# Camera input configuration
CAMERA_INDEX = 0  # change this when camera input changes

# model configuration
MODEL_PATH = "thumb_gesture_model.pkl"   # Saved model file
DATA_PATH  = "thumb_gesture_data.pkl"    # Saved training data file
MIN_SAMPLES_TO_TRAIN = 30               # Minimum labelled samples before first training
RETRAIN_EVERY = 10                      # Retrain model after every 10 new samples
LABEL_MAP = {0: "Thumbs Up", 1: "Thumbs Down", 2: "Unknown"}
LABEL_KEY  = {"u": 0, "d": 1, "x": 2}  # Keyboard shortcuts for labelling

# hand setup with mediapipe
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# feature extractionj
def extract_features(hand_landmarks):
    """
    Flatten all 21 hand landmarks (x, y, z) into a 63-dim feature vector,
    normalised relative to the wrist landmark so the model is
    translation-invariant.
    """
    wrist = hand_landmarks.landmark[0]
    features = []
    for lm in hand_landmarks.landmark:
        features.extend([
            lm.x - wrist.x,
            lm.y - wrist.y,
            lm.z - wrist.z
        ])
    return np.array(features, dtype=np.float32)


# -----------------------------
# Rule-based feedback classifier used before the ML model has enough data
def classify_thumb_rules(hand_landmarks):
    THUMB_TIP = 4
    THUMB_IP  = 3
    INDEX_MCP = 5
    WRIST     = 0

    thumb_tip = hand_landmarks.landmark[THUMB_TIP]
    thumb_ip  = hand_landmarks.landmark[THUMB_IP]
    wrist     = hand_landmarks.landmark[WRIST]
    index_mcp = hand_landmarks.landmark[INDEX_MCP]

    thumb_extended = abs(thumb_tip.x - index_mcp.x) > 0.04
    if not thumb_extended:
        return "Unknown"

    if thumb_tip.y < wrist.y and thumb_tip.y < thumb_ip.y:
        return "Thumbs Up"
    if thumb_tip.y > wrist.y and thumb_tip.y > thumb_ip.y:
        return "Thumbs Down"
    return "Unknown"


# store training data
class TrainingDataStore:
    def __init__(self, path):
        self.path = path
        self.X = []   # feature vectors
        self.y = []   # integer labels
        self.load()

    def add(self, features, label_int):
        self.X.append(features)
        self.y.append(label_int)

    def save(self):
        with open(self.path, "wb") as f:
            pickle.dump({"X": self.X, "y": self.y}, f)

    def load(self):
        if os.path.exists(self.path):
            with open(self.path, "rb") as f:
                data = pickle.load(f)
            self.X = data["X"]
            self.y = data["y"]
            print(f"[DATA] Loaded {len(self.X)} samples from {self.path}")
        else:
            print("[DATA] No existing data file found — starting fresh.")

    @property
    def count(self):
        return len(self.X)

    def class_counts(self):
        counts = {v: 0 for v in LABEL_MAP.values()}
        for label in self.y:
            counts[LABEL_MAP[label]] += 1
        return counts


# ML model
class GestureModel:
    def __init__(self, path):
        self.path = path
        self.model = None
        self.trained = False
        self.load()

    def build(self):
        return Pipeline([
            ("scaler", StandardScaler()),
            ("mlp", MLPClassifier(
                hidden_layer_sizes=(128, 64),
                activation="relu",
                max_iter=500,
                random_state=42,
                early_stopping=True,
                validation_fraction=0.15,
                n_iter_no_change=20,
                verbose=False
            ))
        ])

    def train(self, X, y):
        if len(set(y)) < 2:
            print("[MODEL] Need at least 2 classes to train — collect more data.")
            return False
        self.model = self.build()
        self.model.fit(np.array(X), np.array(y))
        self.trained = True
        self.save()
        print(f"[MODEL] Trained on {len(X)} samples. Model saved.")
        return True

    def predict(self, features):
        if not self.trained or self.model is None:
            return None, 0.0
        proba = self.model.predict_proba([features])[0]
        label_int = int(np.argmax(proba))
        confidence = float(proba[label_int])
        return LABEL_MAP[label_int], confidence

    def save(self):
        with open(self.path, "wb") as f:
            pickle.dump(self.model, f)

    def load(self):
        if os.path.exists(self.path):
            with open(self.path, "rb") as f:
                self.model = pickle.load(f)
            self.trained = True
            print(f"[MODEL] Loaded existing model from {self.path}")
        else:
            print("[MODEL] No existing model — will train once enough data is collected.")


# helpers
def draw_hud(frame, gesture_text, confidence, mode, data_store, samples_since_retrain):
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Semi-transparent dark panel
    cv2.rectangle(overlay, (10, 10), (420, 230), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    color = (50, 255, 50) if "Up" in gesture_text else \
            (50, 50, 255) if "Down" in gesture_text else \
            (180, 180, 180)

    cv2.putText(frame, f"Gesture: {gesture_text}", (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    conf_text = f"Confidence: {confidence:.0%}" if confidence > 0 else "Confidence: N/A (rule-based)"
    cv2.putText(frame, conf_text, (20, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    cv2.putText(frame, f"Mode: {mode}", (20, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (100, 255, 255) if mode == "ML Model" else (255, 200, 50), 1)

    counts = data_store.class_counts()
    cv2.putText(frame, f"Data: UP={counts['Thumbs Up']}  DN={counts['Thumbs Down']}  UNK={counts['Unknown']}",
                (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

    cv2.putText(frame, f"Total samples: {data_store.count}  (next retrain in {max(0, RETRAIN_EVERY - samples_since_retrain)})",
                (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

    # Controls legend
    cv2.putText(frame, "[U]=Label Up  [D]=Label Down  [X]=Label Unknown  [Q]=Quit",
                (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)

    # Flash banner when a label is just added
    return frame


def flash_label_banner(frame, label_text):
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h//2 - 40), (w, h//2 + 40), (30, 30, 30), -1)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
    cv2.putText(frame, f"LABELLED: {label_text}", (w//2 - 160, h//2 + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 160), 3)
    return frame


# main
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    data_store           = TrainingDataStore(DATA_PATH)
    model                = GestureModel(MODEL_PATH)
    samples_since_retrain = 0
    flash_until          = 0   # timestamp until which we show the label banner
    flash_label          = ""
    last_features        = None
    last_landmarks       = None

    # If we have loaded data but no model, do an initial training pass
    if data_store.count >= MIN_SAMPLES_TO_TRAIN and not model.trained:
        print(f"[BOOT] Found {data_store.count} saved samples — running initial training...")
        model.train(data_store.X, data_store.y)

    print("\n=== Thumb Gesture RL Trainer ===")
    print("Show a gesture, then press:")
    print("  U → label as Thumbs Up")
    print("  D → label as Thumbs Down")
    print("  X → label as Unknown")
    print("  Q → quit\n")

    while True:
        success, frame = cap.read()
        if not success:
            print("Failed to read camera input")
            break

        frame     = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results   = hands.process(rgb_frame)

        gesture_text  = "No Hand Detected"
        confidence    = 0.0
        mode          = "No Hand"
        last_features = None
        last_landmarks = None

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                last_features  = extract_features(hand_landmarks)
                last_landmarks = hand_landmarks

                if model.trained:
                    gesture_text, confidence = model.predict(last_features)
                    mode = "ML Model"
                else:
                    gesture_text = classify_thumb_rules(hand_landmarks)
                    confidence   = 0.0
                    mode         = f"Rule-based (need {MIN_SAMPLES_TO_TRAIN - data_store.count} more samples)"

        frame = draw_hud(frame, gesture_text, confidence, mode, data_store, samples_since_retrain)

        # Show flash banner if a label was recently added
        if time.time() < flash_until:
            frame = flash_label_banner(frame, flash_label)

        cv2.imshow("Thumb Gesture RL Trainer", frame)

        key = cv2.waitKey(1) & 0xFF

        # Quit
        if key == ord('q'):
            break

        # Label the current frame
        label_int = None
        if key == ord('u'):
            label_int = 0
        elif key == ord('d'):
            label_int = 1
        elif key == ord('x'):
            label_int = 2

        if label_int is not None:
            if last_features is not None:
                data_store.add(last_features, label_int)
                data_store.save()
                samples_since_retrain += 1
                flash_label = LABEL_MAP[label_int]
                flash_until = time.time() + 0.8
                print(f"[LABEL] Added '{LABEL_MAP[label_int]}' — total samples: {data_store.count}")

                # Train / retrain when enough data is available
                if data_store.count >= MIN_SAMPLES_TO_TRAIN and samples_since_retrain >= RETRAIN_EVERY:
                    print(f"[TRAIN] Retraining on {data_store.count} samples...")
                    model.train(data_store.X, data_store.y)
                    samples_since_retrain = 0

                elif data_store.count == MIN_SAMPLES_TO_TRAIN:
                    # First time we hit the threshold
                    print(f"[TRAIN] Reached {MIN_SAMPLES_TO_TRAIN} samples — running first training...")
                    model.train(data_store.X, data_store.y)
                    samples_since_retrain = 0
            else:
                print("[LABEL] No hand visible — show your hand before labelling.")

    cap.release()
    cv2.destroyAllWindows()

    # Final save
    data_store.save()
    print(f"\n[DONE] Session ended. {data_store.count} total labelled samples saved.")


if __name__ == "__main__":
    main()
