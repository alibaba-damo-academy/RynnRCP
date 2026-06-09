from pygrabber.dshow_graph import FilterGraph
import cv2
import time

devices = FilterGraph().get_input_devices()
print("Available cameras:")
for i, name in enumerate(devices):
    print(f"  {i}: {name}")

# 尝试依次打开并立即释放
for i in range(len(devices)):
    print(f"\nTrying to open camera {i}...")
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f"  [OK] Success")
        cap.release()
    else:
        print(f"  [ERR] Failed")


def test_camera(cam_id, name=""):
    cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"[ERR] Camera {cam_id} ({name}) failed")
        return
    print(f"[OK] Showing Camera {cam_id}: {name}. Press 'q' to quit.")
    cv2.namedWindow(f'Camera {cam_id} - {name}', cv2.WINDOW_NORMAL)

    time.sleep(1)
    timeout_cnt = 10

    while True:
        time.sleep(0.5)
        ret, frame = cap.read()
        timeout_cnt -= 1
        if timeout_cnt <= 0:
            print(f"[ERR] Camera {cam_id} ({name}) timeout")
            break
        if not ret:
            break
        cv2.imshow(f'Camera {cam_id} - {name}', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# 测试每个
for i, name in enumerate(devices):
    input(f"\nPress Enter to test camera {i}: {name}")
    test_camera(i, name)
