import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rcParams
from sklearn.preprocessing import MinMaxScaler
import numpy as np
import matplotlib
from matplotlib.animation import FuncAnimation


# LSTMモデル定義
class LSTMModel(nn.Module):
    def __init__(self, input_size=5, hidden_size=32, num_layers=1):
        super(LSTMModel, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        # LSTM層
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        # 全結合層（減速比予測）
        self.fc = nn.Linear(hidden_size, 2)  # 2つの出力 (Reduction_Ratio_0, Reduction_Ratio_1)

    def forward(self, x):
        # 初期隠れ状態とセル状態
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        # LSTM処理
        out, _ = self.lstm(x, (h0, c0))
        # 最後のタイムステップの出力を取得し、減速比を予測
        out = self.fc(out[:, -1, :])
        return out

# CSVデータ読み込み
data = pd.read_csv('csv/motion/two_pos_red_0218.csv')  # 学習用データのパス
print("学習データの列名:", data.columns)

# 特徴量とターゲットの分離
features = data[['time', 'Ref_0', 'Position_0', 'Ref_1', 'Position_1']].values  # 入力特徴量
targets = data[['Reduction_Ratio_0', 'Reduction_Ratio_1']].values  # ターゲット



# 特徴量とターゲットのスケーリング
scaler_features = MinMaxScaler()
features = scaler_features.fit_transform(features)

scaler_targets = MinMaxScaler()
targets = scaler_targets.fit_transform(targets)

# データ整形
sequence_length = 10
input_size = features.shape[1]
X = []
y = []
for i in range(len(features) - sequence_length + 1):
    X.append(features[i:i+sequence_length])
    y.append(targets[i+sequence_length-1])
X = torch.tensor(X, dtype=torch.float32)
y = torch.tensor(y, dtype=torch.float32)

# モデル初期化
model = LSTMModel(input_size=input_size, hidden_size=64, num_layers=2)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 訓練ループ
num_epochs = 1000
for epoch in range(num_epochs):
    model.zero_grad()
    outputs = model(X)
    loss = criterion(outputs, y)
    loss.backward()
    optimizer.step()
    if epoch % 100 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item()}")

# 新しいデータに対する予測
model.eval()
with torch.no_grad():
    new_data = pd.read_csv('csv/motion/two_pos_trac_20250218_111129.csv')
    new_features = new_data[['time', 'ref_0', 'Position_0', 'ref_1', 'Position_1']].values
    new_features = scaler_features.transform(new_features)

    predictions = []
    for i in range(len(new_features) - sequence_length + 1):
        input_seq = torch.tensor(new_features[i:i+sequence_length], dtype=torch.float32).unsqueeze(0)
        predicted_ratios = model(input_seq)
        predictions.append(predicted_ratios.numpy())

    predictions = np.array(predictions).squeeze(1)
    predictions = scaler_targets.inverse_transform(predictions)  # スケーリングを元に戻す

# フォントを Times New Roman に設定
rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['Times New Roman']  # Times New Roman を指定


# グラフ描画
plt.figure(figsize=(12, 8))

# サブプロット1: Reduction Ratios
plt.subplot(2, 1, 2)  # 2行1列の1番目のグラフ
plt.plot(predictions[:, 0], label="Reduction_Ratio_0", color="blue")
plt.plot(predictions[:, 1], label="Reduction_Ratio_1", color="orange")
plt.title("Predicted Reduction Ratios")
plt.xlabel("Time[s]")
plt.ylabel("Reduction Ratio")
plt.legend()

# サブプロット2: Position Data
plt.subplot(2, 1, 1)  # 2行1列の2番目のグラフ
plt.plot(new_features[sequence_length - 1:, 2], label="Position_0", color="green")  # Position_0
plt.plot(new_features[sequence_length - 1:, 4], label="Position_1", color="red")    # Position_1
plt.title("Position Data")
plt.xlabel("Time[s]")
plt.ylabel("Position")
plt.legend()

# レイアウトを調整
plt.tight_layout()

# グラフをSVG形式で保存
plt.savefig("fig/predicted_reduction_ratios.svg", format="svg")

# グラフを表示
plt.show()

torque = np.linspace(0, 11, 50)

# Speed (RPM) - Concave-down quadratic approximation
vertex = 15
a = (3800 - 2300) / ((0 - vertex)**2 - (12 - vertex)**2)
b = -2 * a * vertex
c = 3800
speed = a * torque**2 + b * torque + c

# Reduction ratios from predicted_ratios
ratios = predictions[:, 0] # Use the first column of predicted_ratios and convert to numpy array

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(8, 6))
line, = ax.plot([], [], color='blue', lw=2)
fill = None

# Set axis labels and limits
font_properties = {'family': 'Times New Roman', 'size': 18}
ax.set_xlabel('Torque [Nm]', fontdict=font_properties)
ax.set_ylabel('Speed [rpm]', fontdict=font_properties)
ax.set_xlim(0, 380)
ax.set_ylim(0, 580)
ax.tick_params(labelsize=16)

# Initialization function for the animation
def init():
    global fill
    line.set_data([], [])
    if fill is not None:
        try:
            fill.remove()
        except ValueError:
            pass
    return line,

# Update function for the animation
def update(frame):
    global fill
    ratio = ratios[frame]
    reduced_speed = speed / ratio
    torque_scaled = torque * ratio
    line.set_data(torque_scaled, reduced_speed)
    if fill is not None:
        try:
            fill.remove()
        except ValueError:
            pass
    fill = ax.fill_between(torque_scaled, reduced_speed, color='blue', alpha=0.3)
    ax.set_title(f"Reduction Ratio: {ratio:.2f}", fontsize=18, family='Times New Roman')
    return line, fill

# Create the animation
ani = FuncAnimation(fig, update, frames=len(ratios), init_func=init, blit=False, interval=50)

# # Save the animation as a video or GIF (optional)
ani.save('ani/predicted_ratios_animation.gif', writer='imagemagick')

# Show the animation
plt.show()