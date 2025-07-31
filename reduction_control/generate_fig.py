import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_lstm_model(input_size, hidden_size, num_layers, output_size=2):
    fig, ax = plt.subplots(figsize=(10, 6))
    
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')

    # 入力層
    ax.text(1, 4, f"Input\n(size={input_size})", ha='center', va='center', fontsize=12, bbox=dict(boxstyle="round", fc="lightblue"))
    ax.arrow(1.5, 4, 1.0, 0, head_width=0.2, head_length=0.2, fc='k', ec='k')

    # LSTM層
    lstm_text = f"LSTM Layer(s)\n(hidden={hidden_size}, layers={num_layers})"
    ax.text(3.5, 4, lstm_text, ha='center', va='center', fontsize=12, bbox=dict(boxstyle="round", fc="lightgreen"))
    ax.arrow(4.5, 4, 1.0, 0, head_width=0.2, head_length=0.2, fc='k', ec='k')

    # 全結合層
    ax.text(6, 4, f"Fully Connected\n(size={output_size})", ha='center', va='center', fontsize=12, bbox=dict(boxstyle="round", fc="lightcoral"))
    ax.arrow(6.7, 4, 1.0, 0, head_width=0.2, head_length=0.2, fc='k', ec='k')

    # 出力層
    ax.text(8.5, 4, "Output\n(Reduction Ratios)", ha='center', va='center', fontsize=12, bbox=dict(boxstyle="round", fc="khaki"))

    # タイトル
    ax.set_title("LSTM Model Architecture", fontsize=16)

    plt.tight_layout()
    plt.show()

# パラメータはあなたのコードと一致させる
draw_lstm_model(input_size=5, hidden_size=64, num_layers=2)
