import h5py
import cv2
import numpy as np

h5_path = "/home/zjy/rgb_action_dataset.hdf5"

def main():
    with h5py.File(h5_path, "r") as f:
        img_group = f["images"]
        ts_group = f["timestamps"]
        act_group = f["actions"]

        frame_keys = sorted(img_group.keys(), key=lambda x: int(x))
        total_frames = len(frame_keys)
        current = 0

        print(f"共有 {total_frames} 帧图像数据。按 d 下一帧，a 上一帧，q 退出。")

        while True:
            key = frame_keys[current]
            img = img_group[key][:]
            ts = ts_group[key][()]
            act = act_group[key][:]

            # img_rgb = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) 
            cv2.imshow("HDF5 Viewer", img)
            print(f"\n🖼 Frame {key}")
            if isinstance(ts, np.ndarray):
                ts_val = ts[0]
            else:
                ts_val = ts
            print(f"  🕒 Timestamp: {ts_val:.6f}")
            print(f"  🎮 Action: position({act[0]:.3f}, {act[1]:.3f}, {act[2]:.3f}), "
                  f"orientation({act[3]:.3f}, {act[4]:.3f}, {act[5]:.3f}, {act[6]:.3f})")

            key = cv2.waitKey(0)

            if key == ord('q'):
                break
            elif key == ord('d') and current < total_frames - 1:
                current += 1
            elif key == ord('a') and current > 0:
                current -= 1

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
