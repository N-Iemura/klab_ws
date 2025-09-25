import tkinter as tk
from tkinter import ttk
import threading
import time

class PIDGainAdjuster:
    def __init__(self, pid_controllers):
        """
        PIDゲイン調整GUI
        
        Args:
            pid_controllers: dict形式 {name: pid_controller_instance}
        """
        self.pid_controllers = pid_controllers
        
        # メインウィンドウ
        self.root = tk.Tk()
        self.root.title("PID Gain Adjuster")
        self.root.geometry("600x500")
        
        # フレーム作成
        self.create_widgets()
        
    def create_widgets(self):
        # タイトル
        title_label = tk.Label(self.root, text="PIDゲイン調整ツール", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # 各PIDコントローラーのスライダー作成
        self.sliders = {}
        
        for name, pid_controller in self.pid_controllers.items():
            # コントローラー名フレーム
            controller_frame = ttk.LabelFrame(self.root, text=name, padding=10)
            controller_frame.pack(fill="x", padx=10, pady=5)
            
            self.sliders[name] = {}
            
            # Kp スライダー
            kp_frame = tk.Frame(controller_frame)
            kp_frame.pack(fill="x", pady=2)
            tk.Label(kp_frame, text="Kp:", width=3).pack(side="left")
            kp_var = tk.DoubleVar(value=pid_controller.kp)
            kp_slider = tk.Scale(kp_frame, from_=0.0, to=50.0, resolution=0.1, 
                               orient="horizontal", variable=kp_var,
                               command=lambda val, n=name, p='kp': self.update_gain(n, p, val))
            kp_slider.pack(side="left", fill="x", expand=True)
            kp_entry = tk.Entry(kp_frame, textvariable=kp_var, width=8)
            kp_entry.pack(side="right")
            kp_entry.bind('<Return>', lambda e, n=name, p='kp', v=kp_var: self.update_gain(n, p, v.get()))
            
            # Ki スライダー
            ki_frame = tk.Frame(controller_frame)
            ki_frame.pack(fill="x", pady=2)
            tk.Label(ki_frame, text="Ki:", width=3).pack(side="left")
            ki_var = tk.DoubleVar(value=pid_controller.ki)
            ki_slider = tk.Scale(ki_frame, from_=0.0, to=5.0, resolution=0.01, 
                               orient="horizontal", variable=ki_var,
                               command=lambda val, n=name, p='ki': self.update_gain(n, p, val))
            ki_slider.pack(side="left", fill="x", expand=True)
            ki_entry = tk.Entry(ki_frame, textvariable=ki_var, width=8)
            ki_entry.pack(side="right")
            ki_entry.bind('<Return>', lambda e, n=name, p='ki', v=ki_var: self.update_gain(n, p, v.get()))
            
            # Kd スライダー
            kd_frame = tk.Frame(controller_frame)
            kd_frame.pack(fill="x", pady=2)
            tk.Label(kd_frame, text="Kd:", width=3).pack(side="left")
            kd_var = tk.DoubleVar(value=pid_controller.kd)
            kd_slider = tk.Scale(kd_frame, from_=0.0, to=10.0, resolution=0.01, 
                               orient="horizontal", variable=kd_var,
                               command=lambda val, n=name, p='kd': self.update_gain(n, p, val))
            kd_slider.pack(side="left", fill="x", expand=True)
            kd_entry = tk.Entry(kd_frame, textvariable=kd_var, width=8)
            kd_entry.pack(side="right")
            kd_entry.bind('<Return>', lambda e, n=name, p='kd', v=kd_var: self.update_gain(n, p, v.get()))
            
            self.sliders[name] = {
                'kp': {'var': kp_var, 'slider': kp_slider},
                'ki': {'var': ki_var, 'slider': ki_slider},
                'kd': {'var': kd_var, 'slider': kd_slider}
            }
        
        # ボタンフレーム
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        # リセットボタン
        reset_button = tk.Button(button_frame, text="全リセット", command=self.reset_all)
        reset_button.pack(side="left", padx=5)
        
        # 保存ボタン
        save_button = tk.Button(button_frame, text="設定保存", command=self.save_gains)
        save_button.pack(side="left", padx=5)
        
        # 読込ボタン
        load_button = tk.Button(button_frame, text="設定読込", command=self.load_gains)
        load_button.pack(side="left", padx=5)
        
        # 現在値表示
        self.status_label = tk.Label(self.root, text="調整中...", font=("Arial", 10))
        self.status_label.pack(pady=10)
        
    def update_gain(self, controller_name, gain_type, value):
        """ゲイン値を更新"""
        try:
            value = float(value)
            controller = self.pid_controllers[controller_name]
            
            if gain_type == 'kp':
                controller.kp = value
            elif gain_type == 'ki':
                controller.ki = value
            elif gain_type == 'kd':
                controller.kd = value
                
            # ステータス更新
            self.status_label.config(text=f"{controller_name} {gain_type.upper()}={value:.3f} に更新")
            
        except ValueError:
            print(f"無効な値: {value}")
            
    def reset_all(self):
        """全ゲインをリセット"""
        default_gains = {
            'Motor0_Position': {'kp': 20.0, 'ki': 0.1, 'kd': 2.0},
            'Motor0_Velocity': {'kp': 0.5, 'ki': 0.05, 'kd': 0.01},
            'Motor1_Position': {'kp': 18.0, 'ki': 0.08, 'kd': 1.8},
            'Motor1_Velocity': {'kp': 0.4, 'ki': 0.04, 'kd': 0.008}
        }
        
        for name, gains in default_gains.items():
            if name in self.pid_controllers:
                for gain_type, value in gains.items():
                    self.sliders[name][gain_type]['var'].set(value)
                    self.update_gain(name, gain_type, value)
                    
        self.status_label.config(text="全ゲインをデフォルト値にリセットしました")
        
    def save_gains(self):
        """現在のゲイン設定をファイルに保存"""
        filename = f"pid_gains_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        
        with open(filename, 'w') as f:
            f.write("# PID Gain Settings\n")
            f.write(f"# Saved at: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            for name, controller in self.pid_controllers.items():
                f.write(f"[{name}]\n")
                f.write(f"Kp = {controller.kp:.6f}\n")
                f.write(f"Ki = {controller.ki:.6f}\n")
                f.write(f"Kd = {controller.kd:.6f}\n\n")
                
        self.status_label.config(text=f"設定を保存しました: {filename}")
        
    def load_gains(self):
        """ファイルからゲイン設定を読み込み"""
        # 簡単な実装例（ファイルダイアログなどを追加可能）
        try:
            filename = "pid_gains.txt"  # デフォルトファイル名
            gains_data = {}
            
            with open(filename, 'r') as f:
                current_controller = None
                for line in f:
                    line = line.strip()
                    if line.startswith('[') and line.endswith(']'):
                        current_controller = line[1:-1]
                        gains_data[current_controller] = {}
                    elif '=' in line and current_controller:
                        key, value = line.split('=', 1)
                        key = key.strip().lower()
                        value = float(value.strip())
                        gains_data[current_controller][key] = value
            
            # 読み込んだ値をGUIに反映
            for name, gains in gains_data.items():
                if name in self.sliders:
                    for gain_type, value in gains.items():
                        if gain_type in self.sliders[name]:
                            self.sliders[name][gain_type]['var'].set(value)
                            self.update_gain(name, gain_type, value)
                            
            self.status_label.config(text=f"設定を読み込みました: {filename}")
            
        except FileNotFoundError:
            self.status_label.config(text="設定ファイルが見つかりません")
        except Exception as e:
            self.status_label.config(text=f"読み込みエラー: {e}")
            
    def start(self):
        """GUI開始"""
        self.root.mainloop()

# 使用例
if __name__ == "__main__":
    # ダミーのPIDコントローラー（実際には torque_pid_control.py から import）
    class DummyPID:
        def __init__(self, kp, ki, kd):
            self.kp = kp
            self.ki = ki
            self.kd = kd
    
    controllers = {
        'Motor0_Position': DummyPID(20.0, 0.1, 2.0),
        'Motor0_Velocity': DummyPID(0.5, 0.05, 0.01),
        'Motor1_Position': DummyPID(18.0, 0.08, 1.8),
        'Motor1_Velocity': DummyPID(0.4, 0.04, 0.008)
    }
    
    adjuster = PIDGainAdjuster(controllers)
    adjuster.start()