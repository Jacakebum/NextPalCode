from maix import camera, display, image, nn, app, time, touchscreen, pinmap, err, uart

detector = nn.HandLandmarks(model="/root/models/hand_landmarks.mud")
# detector = nn.HandLandmarks(model="/root/models/hand_landmarks_bf16.mud")
landmarks_rel = False
CUSTOM_MODEL_CONFIGS = [
    {
        "name": "LEFT",
        "output_gesture": "LEFT",
        "candidates": [
            "left/model_260320.mud",
            "./left/model_260320.mud",
            "/maixapp/apps/gesture-classifier/left/model_260320.mud",
            "/maixapp/apps/gesture_classifier/left/model_260320.mud",
        ],
    },
]

cam = camera.Camera(320, 224, detector.input_format())
disp = display.Display()

ts = touchscreen.TouchScreen()

USE_UART = True
CONFIDENCE_THRESHOLD = 0.75
SEND_INTERVAL_MS = 200
REPEAT_INTERVAL_MS = 1200
last_send_ms = 0
last_sent_cmd = None

err.check_raise(pinmap.set_pin_function("A16", "UART0_TX"), "set A16 failed")
err.check_raise(pinmap.set_pin_function("A17", "UART0_RX"), "set A17 failed")
serial = uart.UART("/dev/ttyS0", 115200)

UART_CMD_MAP = {
    "five": ("12",),
    "thumbUp": ("37",),
    "ok": ("11", "03"),
    "three": ("11", "08"),
    "one": ("11", "02"),
}


def now_ms():
    return time.ticks_ms()


def wait_ms(ms):
    start = now_ms()
    while now_ms() - start < ms:
        pass


def send_ascii(ascii_code):
    if ascii_code is None or len(ascii_code) != 2:
        print("INVALID ASCII:", ascii_code)
        return False
    if not ascii_code.isdigit():
        print("INVALID ASCII:", ascii_code)
        return False

    if not USE_UART:
        print("ASCII:", ascii_code)
        return True

    serial.write_str(ascii_code)
    print("SEND ASCII:", ascii_code)
    return True


def send_cmd(gesture_name):
    global last_send_ms, last_sent_cmd
    cmd_seq = UART_CMD_MAP.get(gesture_name)
    if cmd_seq is None:
        return

    cur = now_ms()
    min_interval = REPEAT_INTERVAL_MS if cmd_seq == last_sent_cmd else SEND_INTERVAL_MS
    if cur - last_send_ms < min_interval:
        return

    for index, ascii_code in enumerate(cmd_seq):
        if not send_ascii(ascii_code):
            return
        if index < len(cmd_seq) - 1:
            wait_ms(60)

    last_send_ms = now_ms()
    last_sent_cmd = cmd_seq


def get_ascii_code(gesture_name):
    cmd_seq = UART_CMD_MAP.get(gesture_name)
    if cmd_seq is None:
        return None
    return "+".join(cmd_seq)

# Loading screen
img = cam.read()
img.draw_string(100, 112, "Loading...\nwait up to 10s", color=image.COLOR_GREEN)
disp.show(img)

# Train LinearSVC
import numpy as np

from LinearSVC import LinearSVC, LinearSVCManager

from contextlib import contextmanager

@contextmanager
def timer(name):
    import time
    result = {'passed': 0.}
    start = time.time()
    yield result
    end = time.time()
    passed = end - start
    result['passed'] = passed
    print(f"{name} 耗时: {passed:.6f} 秒")


print("hello")
name_classes = ("one", "five", "fist", "ok", "heartSingle", "yeah", "three", "four", "six", "Iloveyou", "gun", "thumbUp", "nine", "pink")
last_train_time = 0

npzfile = np.load("trainSets.npz")
X_train = npzfile["X"]
y_train = npzfile["y"]
assert len(X_train) == len(y_train)
print(f"X_train_len: {len(X_train)}")

if 1:
    with timer("加载") as r:
        clfm = LinearSVCManager(LinearSVC.load("clf_dump.npz"), X_train, y_train, pretrained=True)
    last_train_time = r['passed']
else:
    mask_lt_4 = y_train < 4
    mask_ge_4 = y_train >= 4
    with timer("训练前半部分") as r:
        clfm = LinearSVCManager(LinearSVC(C=1.0, learning_rate=0.01, max_iter=100), X_train[mask_lt_4], y_train[mask_lt_4])
    last_train_time = r['passed']

with timer("回归"):
    labels, confs = clfm.test(clfm.samples[0])
    recall_count = len(clfm.samples[1])
    right_count = np.sum(labels == clfm.samples[1])
    print(f"right/all= {right_count}/{recall_count}, acc: {right_count/recall_count}")

print(type(clfm.clf))


def preprocess(hand_landmarks, is_left=False, boundary=(1, 1, 1)):
    hand_landmarks = np.array(hand_landmarks).reshape((21, -1))
    vector = hand_landmarks[:, :2]
    vector = vector[1:] - vector[0]
    vector = vector.astype('float64') / boundary[:vector.shape[1]]
    if not is_left:  # mirror
        vector[:, 0] *= -1
    return vector


def is_in_button(px, py, btn_rect):
    bx, by, bw, bh = btn_rect
    return bx <= px <= bx + bw and by <= py <= by + bh


# Home button on image coordinates
HOME_BTN = (8, 8, 64, 28)
HOME_LABEL_X = 16
HOME_LABEL_Y = 14

# Skeleton display size: smaller than original to avoid blocking view
LANDMARK_RADIUS = 2
LANDMARK_LINE_THICKNESS = 5

# main loop
class_nums_changing = False
home_pressed_last = False

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.7, iou_th=0.45, conf_th2=0.8, landmarks_rel=landmarks_rel)

    # draw home button first
    img.draw_rect(HOME_BTN[0], HOME_BTN[1], HOME_BTN[2], HOME_BTN[3], color=image.COLOR_WHITE, thickness=1)
    img.draw_string(HOME_LABEL_X, HOME_LABEL_Y, "Home", color=image.COLOR_WHITE, scale=1.0, thickness=1)

    for obj in objs:
        hand_landmarks = preprocess(obj.points[8:8 + 21 * 3], obj.class_id == 0, (img.width(), img.height(), 1))
        features = np.array([hand_landmarks.flatten()])
        class_idx, pred_conf = clfm.test(features)
        class_idx, pred_conf = class_idx[0], pred_conf[0]
        gesture_name = name_classes[class_idx]
        ascii_code = get_ascii_code(gesture_name) or "--"
        uart_ready = ascii_code != "--" and pred_conf >= CONFIDENCE_THRESHOLD

        msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}\n{gesture_name}({class_idx})={pred_conf * 100:.2f}%\nASCII: {ascii_code}'
        img.draw_string(obj.points[0], obj.points[1], msg,
                        color=image.COLOR_GREEN if uart_ready else image.COLOR_RED,
                        scale=1.2, thickness=1)

        if uart_ready:
            send_cmd(gesture_name)

        # smaller hand skeleton + box kept
        detector.draw_hand(img, obj.class_id, obj.points, LANDMARK_RADIUS, LANDMARK_LINE_THICKNESS, box=True)

        if landmarks_rel:
            img.draw_rect(0, 0, detector.input_width(detect=False), detector.input_height(detect=False), color=image.COLOR_YELLOW)
            for i in range(21):
                x = obj.points[8 + 21 * 3 + i * 2]
                y = obj.points[8 + 21 * 3 + i * 2 + 1]
                img.draw_circle(x, y, 2, color=image.COLOR_YELLOW)

    current_n_classes = len(clfm.clf.classes)

    get_color = lambda n: image.COLOR_GREEN if current_n_classes == n else image.COLOR_RED
    img.draw_circle(300, 20, 30, color=get_color(14))
    img.draw_string(300 - 22, 20 - 18, "class 14", color=get_color(14), scale=1.0, thickness=1)
    img.draw_circle(300, 224 - 1 - 20, 30, color=get_color(4))
    img.draw_string(300 - 22, 224 - 1 - 20 - 18, "class 4", color=get_color(4), scale=1.0, thickness=1)

    tx, ty, pressed = ts.read()
    tx = int(tx / disp.width() * img.width())
    ty = int(ty / disp.height() * img.height())

    # home button handling: trigger once on touch press
    if pressed and is_in_button(tx, ty, HOME_BTN):
        if not home_pressed_last:
            app.set_exit_flag(True)
        home_pressed_last = True
        disp.show(img)
        continue
    else:
        home_pressed_last = False

    if tx >= 300 - 30:
        if ty <= 20 + 30:
            if pressed:
                if not class_nums_changing and current_n_classes == 4:
                    class_nums_changing = True
                if class_nums_changing:
                    img.draw_string(30, 112, "Release to upgrade to class 14\n and please wait for Training be done.", color=image.COLOR_RED, scale=1.0, thickness=1)
            else:
                if class_nums_changing:
                    class_nums_changing = False
                    with timer("训练后半部分") as r:
                        mask_ge_4 = y_train >= 4
                        clfm.add(X_train[mask_ge_4], y_train[mask_ge_4])
                    last_train_time = r['passed']
                    print("success changed to 14")
        elif ty >= 224 - 1 - 20 - 30:
            if pressed:
                if not class_nums_changing and current_n_classes == 14:
                    class_nums_changing = True
                if class_nums_changing:
                    img.draw_string(30, 112, "Release to retrain to class 4\n and please wait for Training be done.", color=image.COLOR_RED, scale=1.0, thickness=1)
            else:
                if class_nums_changing:
                    class_nums_changing = False
                    with timer("移除后半部分") as r:
                        mask_ge_4 = clfm.samples[1] >= 4
                        indices_ge_4 = np.where(mask_ge_4)[0]
                        clfm.rm(indices_ge_4)
                    last_train_time = r['passed']
                    print("success changed to 4")
        elif pressed:
            img.draw_string(30, 112, "Press Red circle to make it\n Green(active).", color=image.COLOR_RED, scale=1.0, thickness=1)
            img.draw_string(0, 0, f'last_train_time= {last_train_time:.6f}s', color=image.COLOR_GREEN, scale=1.0, thickness=1)
    elif pressed:
        img.draw_string(30, 112, "Press Red circle to make it\n Green(active).", color=image.COLOR_RED, scale=1.0, thickness=1)
        img.draw_string(0, 0, ','.join(name_classes[:4]), color=image.COLOR_GREEN, scale=1.0, thickness=1)
        img.draw_string(0, 20, '\n'.join(name_classes[4:]), color=image.COLOR_YELLOW if current_n_classes == 4 else image.COLOR_GREEN, scale=1.0, thickness=1)

    disp.show(img)
