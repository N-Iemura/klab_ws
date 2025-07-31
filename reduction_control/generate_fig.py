import matplotlib.pyplot as plt
import networkx as nx
from matplotlib import rcParams

# フォント設定 (Times New Roman)
rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['Times New Roman']

# グラフの作成
G = nx.DiGraph()

# ノードの追加（ラベルと形状を指定）
nodes = [
    ("Input", {"label": "Input\n(batch_size, 10, 5)"}),
    ("LSTM1", {"label": "LSTM Layer 1\nHidden Size: 64"}),
    ("LSTM2", {"label": "LSTM Layer 2\nHidden Size: 64"}),
    ("Last", {"label": "Last Timestep\n(batch_size, 64)"}),
    ("FC", {"label": "Fully Connected\nOutput: 2"}),
    ("Output", {"label": "Output\nReduction_Ratio_0,1"})
]
G.add_nodes_from(nodes)

# エッジの追加
edges = [
    ("Input", "LSTM1"),
    ("LSTM1", "LSTM2"),
    ("LSTM2", "Last"),
    ("Last", "FC"),
    ("FC", "Output")
]
G.add_edges_from(edges)

# レイアウトの設定（縦に並べる）
pos = {
    "Input": (0, 5),
    "LSTM1": (0, 4),
    "LSTM2": (0, 3),
    "Last": (0, 2),
    "FC": (0, 1),
    "Output": (0, 0)
}

# グラフの描画
plt.figure(figsize=(6, 8))
nx.draw(
    G, pos,
    with_labels=True,
    labels={node: data["label"] for node, data in G.nodes(data=True)},
    node_shape="s",  # 矩形ノード
    node_size=5000,  # ノードの大きさ
    node_color="lightblue",
    font_size=10,
    font_family="Times New Roman",
    arrows=True,
    arrowstyle="->",
    arrowsize=20
)

# グラフをSVG形式で保存
plt.savefig("lstm_model_structure_networkx.svg", format="svg", bbox_inches="tight")
plt.show()