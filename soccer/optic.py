import cv2
import numpy as np
import csv

# ビデオキャプチャの初期化
cap = cv2.VideoCapture(0)

# Shi-Tomasiコーナー検出のパラメータ
feature_params = dict(maxCorners=5, qualityLevel=0.01, minDistance=7, blockSize=7)

# Lucas-Kanade法のパラメータ
lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# ランダムな色を生成
color = np.random.randint(0, 255, (200, 3))

# 最初のフレームの処理
end_flag, frame = cap.read()
if not end_flag:
    print("Error: Could not read the video file.")
    cap.release()
    exit()

gray_prev = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
feature_prev = cv2.goodFeaturesToTrack(gray_prev, mask=None, **feature_params)
mask = np.zeros_like(frame)

# CSVファイルの準備
with open('motion_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Frame', 'Feature_ID', 'X', 'Y', 'Vx', 'Vy'])

    frame_count = 0
    threshold = 2.0  # 速度の閾値

    while end_flag:
        # 次のフレームを読み込む
        end_flag, frame = cap.read()
        if not end_flag:
            break

        # グレースケールに変換
        gray_next = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 特徴点を追跡
        feature_next, status, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray_next, feature_prev, None, **lk_params)

        # 追跡に成功した特徴点を選別
        good_new = feature_next[status == 1]
        good_old = feature_prev[status == 1]

        # 速度を計算し、CSVに書き込む
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            vx, vy = a - c, b - d
            speed = np.sqrt(vx**2 + vy**2)
            if speed > threshold:
                writer.writerow([frame_count, i, a, b, vx, vy])

                # 描画のために新しい点と古い点を描く
                frame = cv2.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
                frame = cv2.putText(frame, str(i), (int(a), int(b)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color[i].tolist(), 2)
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)

        # フレームにマスクを重ねる
        img = cv2.add(frame, mask)

        # 画像を表示
        cv2.imshow('frame', img)

        # 'q'キーが押されたらループを終了
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

        # 次のフレームの準備
        gray_prev = gray_next.copy()
        feature_prev = good_new.reshape(-1, 1, 2)
        frame_count += 1

cap.release()
cv2.destroyAllWindows()