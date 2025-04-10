import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import matplotlib.pyplot as plt

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
data = pd.read_csv('/home/naga/klab_ws/reduction_control/csv/motion/two_pos_red_0218.csv')  # 学習用データのパス
print("学習データの列名:", data.columns)

# 特徴量とターゲットの分離
features = data[['time', 'Ref_0', 'Position_0', 'Ref_1', 'Position_1']].values  # 入力特徴量
targets = data[['Reduction_Ratio_0', 'Reduction_Ratio_1']].values  # ターゲット

# データ整形
sequence_length = 3  # タイムステップ数
input_size = features.shape[1]  # 特徴量数
X = []
y = []
for i in range(len(features) - sequence_length + 1):
    X.append(features[i:i+sequence_length])
    y.append(targets[i+sequence_length-1])
X = torch.tensor(X, dtype=torch.float32)
y = torch.tensor(y, dtype=torch.float32)

# モデル初期化
model = LSTMModel(input_size=input_size, hidden_size=32, num_layers=1)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.01)

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

# 新しい軌道データに対する予測
model.eval()
with torch.no_grad():
    # 新しい軌道データ（Reduction_Ratio_0, Reduction_Ratio_1 がない）
    new_data = pd.read_csv('/home/naga/klab_ws/reduction_control/csv/motion/two_pos_trac_20250218_110915.csv')  # 新しいデータのパス
    print("新しいデータの列名:", new_data.columns)

    # 必要な列のみを選択
    new_features = new_data[['time', 'ref_0', 'Position_0', 'ref_1', 'Position_1']].values

    # シーケンスごとにスライドして予測
    predictions = []
    for i in range(len(new_features) - sequence_length + 1):
        input_seq = torch.tensor(new_features[i:i+sequence_length], dtype=torch.float32).unsqueeze(0)  # [1, シーケンス長, 特徴量数]
        predicted_ratios = model(input_seq)
        predictions.append(predicted_ratios.numpy())

    # 結果を表示
    predictions = torch.tensor(predictions).squeeze(1).numpy()  # [サンプル数, 2]
    print("予測減速比 (Reduction_Ratio_0, Reduction_Ratio_1):")
    print(predictions)

    # グラフ描画
    plt.figure(figsize=(10, 6))
    plt.plot(predictions[:, 0], label="Reduction_Ratio_0", color="blue")
    plt.plot(predictions[:, 1], label="Reduction_Ratio_1", color="orange")
    plt.title("Predicted Reduction Ratios")
    plt.xlabel("Time Step")
    plt.ylabel("Reduction Ratio")
    plt.legend()
    plt.grid()
    plt.show()