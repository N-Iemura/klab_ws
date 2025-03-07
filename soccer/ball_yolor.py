import cv2
import torch
import numpy as np
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import non_max_suppression, scale_coords

# デバイスの設定
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# モデルの読み込み
model = attempt_load('path/to/yolor_p6.pt', map_location=device)  # モデルのパスを指定
model.to(device).eval()

# 入力画像の読み込み
img = cv2.imread('path/to/input_image.jpg')  # 入力画像のパスを指定
img = letterbox(img, new_shape=640)[0]  # 画像のリサイズ
img = img[:, :, ::-1].transpose(2, 0, 1)  # BGRからRGBへ変換、次元の転置
img = np.ascontiguousarray(img)
img = torch.from_numpy(img).to(device)
img = img.float()  # uint8からfloat32へ変換
img /= 255.0  # 0 - 255を0.0 - 1.0に正規化
if img.ndimension() == 3:
    img = img.unsqueeze(0)

# 推論
with torch.no_grad():
    pred = model(img)[0]

# NMS（Non-Maximum Suppression）の適用
pred = non_max_suppression(pred, 0.4, 0.5)

# 推論結果の描画
for det in pred:
    if len(det):
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img.shape[2:]).round()
        for *xyxy, conf, cls in reversed(det):
            # ボールを検出した場合（クラスIDは事前に確認してください）
            if int(cls) == 32:  # 仮にボールのクラスIDが32だと仮定
                c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                cv2.rectangle(img, c1, c2, (0, 255, 0), 2)

# 結果の表示
cv2.imshow('Result', img)
cv2.waitKey(0)
cv2.destroyAllWindows()