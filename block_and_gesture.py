import cv2
import mediapipe as mp
import numpy as np
import time

# -----------------------------
# CAMERA SETTINGS
# -----------------------------

BLOCK_CAMERA = 0
GESTURE_CAMERA = 2


# -----------------------------
# MEDIAPIPE HAND SETUP
# -----------------------------

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)


# -----------------------------
# THUMB RULE CLASSIFIER
# -----------------------------

def classify_thumb(hand_landmarks):

    THUMB_TIP = 4
    THUMB_IP = 3
    WRIST = 0
    INDEX_MCP = 5

    thumb_tip = hand_landmarks.landmark[THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[THUMB_IP]
    wrist = hand_landmarks.landmark[WRIST]
    index_mcp = hand_landmarks.landmark[INDEX_MCP]

    thumb_extended = abs(thumb_tip.x - index_mcp.x) > 0.04

    if not thumb_extended:
        return "Unknown"

    if thumb_tip.y < wrist.y and thumb_tip.y < thumb_ip.y:
        return "Thumbs Up"

    if thumb_tip.y > wrist.y and thumb_tip.y > thumb_ip.y:
        return "Thumbs Down"

    return "Unknown"


# -----------------------------
# WAIT FOR GESTURE FUNCTION
# -----------------------------

def wait_for_gesture():

    cap = cv2.VideoCapture(GESTURE_CAMERA)

    print("Waiting for gesture...")

    while True:

        success, frame = cap.read()

        if not success:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = hands.process(rgb)

        gesture = "No Hand"

        if results.multi_hand_landmarks:

            for hand_landmarks in results.multi_hand_landmarks:

                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                gesture = classify_thumb(hand_landmarks)

        cv2.putText(
            frame,
            f"Gesture: {gesture}",
            (30,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

        cv2.imshow("Gesture Camera", frame)

        if gesture == "Thumbs Up":
            cap.release()
            cv2.destroyWindow("Gesture Camera")
            return "Thumbs Up"

        if gesture == "Thumbs Down":
            cap.release()
            cv2.destroyWindow("Gesture Camera")
            return "Thumbs Down"

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    return "Unknown"


# -----------------------------
# BLOCK COLOUR DETECTION
# -----------------------------

def detect_blocks():

    cap = cv2.VideoCapture(BLOCK_CAMERA)

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        colors = {

            "Red": [
                ((0,120,70),(10,255,255)),
                ((170,120,70),(180,255,255))
            ],

            "Blue":[((100,120,70),(130,255,255))],

            "Yellow":[((20,120,70),(35,255,255))],

            "Green":[((40,70,70),(85,255,255))]
        }

        detected_color = None

        for color_name, ranges in colors.items():

            mask = None

            for lower,upper in ranges:

                m = cv2.inRange(hsv,np.array(lower),np.array(upper))

                mask = m if mask is None else mask | m

            contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:

                area = cv2.contourArea(cnt)

                if area > 800:

                    x,y,w,h = cv2.boundingRect(cnt)

                    cx = x + w//2
                    cy = y + h//2

                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

                    cv2.putText(
                        frame,
                        f"{color_name} block",
                        (x,y-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0,255,0),
                        2
                    )

                    detected_color = color_name

        cv2.imshow("Block Camera",frame)

        key = cv2.waitKey(1)

        if detected_color is not None:

            print(f"Detected block: {detected_color}")

            gesture = wait_for_gesture()

            print(f"Gesture result: {gesture}")

            if gesture == "Thumbs Up":
                print(f"CONFIRMED {detected_color} block")

            elif gesture == "Thumbs Down":
                print(f"REJECTED {detected_color} block")

            time.sleep(2)

        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# -----------------------------
# MAIN
# -----------------------------

if __name__ == "__main__":

    print("Starting block + gesture system")

    detect_blocks()
