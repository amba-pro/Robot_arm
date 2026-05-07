# robot_client.py
import socket
import threading
import time
import json
import os
import tkinter as tk
from tkinter import messagebox
import customtkinter as ctk
from datetime import datetime

# --- Настройки сети ---
BROADCAST_PORT = 50000
COMMAND_PORT = 50001
DISCOVERY_MSG = b"DISCOVER_ROBOHAND"
HISTORY_FILE = "robot_history.json"

# --- Настройка внешнего вида ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")
tcpMessageSize = 8

class RobotClientApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("🤖 Робот-манипулятор — Умный Клиент")
        self.geometry("700x800")
        self.resizable(True, True)

        self.found_robots = {}
        self.connection_socket = None
        self.history = self.load_history()

        self.create_widgets()

    def create_widgets(self):
        # === Header ===
        header = ctk.CTkLabel(self, text="🔍 Умный клиент для робота-манипулятора", font=ctk.CTkFont(size=20, weight="bold"))
        header.pack(pady=10)

        # === Поиск ===
        search_frame = ctk.CTkFrame(self)
        search_frame.pack(pady=10, padx=20, fill="x")

        self.search_btn = ctk.CTkButton(search_frame, text="🔄 Поиск роботов", command=self.start_discovery)
        self.search_btn.pack(side="left", padx=10, pady=10)

        manual_frame = ctk.CTkFrame(search_frame)
        manual_frame.pack(side="right", padx=10)

        self.manual_ip = ctk.CTkEntry(manual_frame, placeholder_text="IP вручную", width=150)
        self.manual_ip.pack(side="left", padx=5)
        ctk.CTkButton(manual_frame, text="➕", width=40, command=self.manual_connect).pack(side="left")

        # === Вкладки ===
        self.tabview = ctk.CTkTabview(self)
        self.tabview.pack(fill="both", expand=True, padx=20, pady=10)

        # --- Вкладка 1: Подключение ---
        tab1 = self.tabview.add("🔌 Подключение")

        ctk.CTkLabel(tab1, text="🌐 Найденные роботы:").pack(anchor="w", padx=10, pady=(10, 5))
        self.robot_listbox = tk.Listbox(tab1, bg="#2a2a2a", fg="white", selectbackground="#3a7ebf", height=8, font=("Consolas", 12))
        self.robot_listbox.pack(fill="x", padx=10, pady=5)
        self.robot_listbox.bind("<Double-1>", lambda e: self.connect_to_robot())

        ctk.CTkLabel(tab1, text="🕒 История подключений:").pack(anchor="w", padx=10, pady=(10, 5))
        self.history_listbox = tk.Listbox(tab1, bg="#2a2a2a", fg="lightgray", height=6, font=("Consolas", 12))
        self.history_listbox.pack(fill="x", padx=10, pady=5)
        self.history_listbox.bind("<Double-1>", lambda e: self.connect_to_selected_history())

        btn_frame = ctk.CTkFrame(tab1)
        btn_frame.pack(pady=10)

        self.connect_btn = ctk.CTkButton(btn_frame, text="Подключиться", command=self.connect_to_robot)
        self.connect_btn.pack(side="left", padx=5)

        self.disconnect_btn = ctk.CTkButton(btn_frame, text="Отключиться", state="disabled", command=self.disconnect)
        self.disconnect_btn.pack(side="left", padx=5)

        # --- Вкладка 2: Управление ---
        tab2 = self.tabview.add("⚙️ Углы сервоприводов")

        mirror_frame = ctk.CTkFrame(tab2)
        mirror_frame.pack(pady=10, padx=20, fill="x")
        self.mirror_enabled = ctk.BooleanVar(value=True)
        self.mirror_checkbox = ctk.CTkCheckBox(mirror_frame, text="🔁 Зеркалировать Серво 1 → Серво 2", variable=self.mirror_enabled)
        self.mirror_checkbox.pack(side="left", padx=10)

        self.angle_sliders = []
        self.angle_bars = []
        self.angle_labels = []

        for i in range(tcpMessageSize):
            frame = ctk.CTkFrame(tab2)
            frame.pack(fill="x", padx=20, pady=5)

            label = ctk.CTkLabel(frame, text=f"Серво {i+1}:", width=80, anchor="w")
            label.pack(side="left")

            slider = ctk.CTkSlider(frame, from_=0, to=180, number_of_steps=180)
            slider.set(90)
            slider.pack(side="left", fill="x", expand=True, padx=10)
            self.angle_sliders.append(slider)

            bar = ctk.CTkProgressBar(frame, width=100)
            bar.set(90 / 180.0)
            bar.pack(side="left", padx=5)
            self.angle_bars.append(bar)

            val_label = ctk.CTkLabel(frame, text="90°", width=40)
            val_label.pack(side="left")
            self.angle_labels.append(val_label)

        # --- Обновление UI и зеркалирование ---
        def make_updater(idx):
            def update(val):
                try:
                    deg = int(float(val))
                    self.angle_labels[idx].configure(text=f"{deg}°")
                    self.angle_bars[idx].set(deg / 180.0)

                    # Зеркалирование: Серво 1 → Серво 2
                    if idx == 0 and self.mirror_enabled.get():
                        mirror_val = 180 - deg
                        # Блокируем рекурсию через временное отключение команды
                        self.angle_sliders[1].configure(command=None)
                        self.angle_sliders[1].set(mirror_val)
                        self.angle_labels[1].configure(text=f"{mirror_val}°")
                        self.angle_bars[1].set(mirror_val / 180.0)
                        self.angle_sliders[1].configure(command=make_updater(1))
                except Exception as e:
                    print(f"Ошибка в слайдере {idx}: {e}")
            return update

        for i in range(tcpMessageSize):
            self.angle_sliders[i].configure(command=make_updater(i))

        # Инициализация зеркала
        make_updater(0)(90)

        # === Лог действий ===
        self.log_text = ctk.CTkTextbox(self, height=80, font=("Consolas", 10))
        self.log_text.pack(fill="x", padx=20, pady=10)
        self.log("✅ Готов к работе. Нажмите 'Поиск роботов'.")

        # === Статус-бар ===
        self.status_label = ctk.CTkLabel(self, text="Готов", fg_color="transparent", text_color="gray")
        self.status_label.pack(side="bottom", pady=5)

        # Запуск периодической отправки
        self.send_angles_periodically()

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")

    def start_discovery(self):
        self.found_robots.clear()
        self.robot_listbox.delete(0, tk.END)
        self.status_label.configure(text="Поиск...")

        thread = threading.Thread(target=self.discover_robots, daemon=True)
        thread.start()

    def discover_robots(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(0.3)
            sock.bind(("", BROADCAST_PORT))

            start_time = time.time()
            while time.time() - start_time < 3:
                sock.sendto(DISCOVERY_MSG, ('<broadcast>', BROADCAST_PORT))
                try:
                    while True:
                        data, addr = sock.recvfrom(1024)
                        msg = data.decode().strip()
                        if msg.startswith("FOUND_ROBOHAND:"):
                            parts = msg.split(":", 3)
                            ip = parts[1]
                            name = parts[2] if len(parts) > 2 else f"Робот {ip.split('.')[-1]}"
                            if ip not in self.found_robots:
                                self.found_robots[ip] = (time.time(), name, addr[0])
                                self.robot_listbox.insert(tk.END, f"{name} — {ip} ({addr[0]})")
                except socket.timeout:
                    continue
        except Exception as e:
            self.log(f"❌ Ошибка поиска: {e}")
        finally:
            sock.close()
            self.status_label.configure(text="Поиск завершён" if self.found_robots else "Роботы не найдены")

    def manual_connect(self):
        ip = self.manual_ip.get().strip()
        if not ip:
            messagebox.showwarning("⚠️", "Введите IP-адрес!")
            return
        name = f"Ручной ввод: {ip}"
        self.connect_to_ip(ip, name)

    def connect_to_robot(self):
        selection = self.robot_listbox.curselection()
        if not selection:
            messagebox.showwarning("⚠️", "Выберите робота из списка!")
            return
        text = self.robot_listbox.get(selection[0])
        ip = text.split("—")[1].strip().split()[0]
        name = text.split("—")[0].strip()
        self.connect_to_ip(ip, name)

    def connect_to_selected_history(self):
        selection = self.history_listbox.curselection()
        if not selection:
            return
        text = self.history_listbox.get(selection[0])
        ip = text.split("—")[1].strip().split()[0]
        name = text.split("—")[0].strip()
        self.connect_to_ip(ip, name)

    def connect_to_ip(self, ip, name="Неизвестный"):
        if self.connection_socket:
            self.disconnect()

        self.connection_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection_socket.settimeout(5)

        try:
            self.connection_socket.connect((ip, COMMAND_PORT))
            self.log(f"✅ Подключено к {name} ({ip})")
            self.status_label.configure(text=f"Подключено к {name}")
            self.connect_btn.configure(state="disabled")
            self.disconnect_btn.configure(state="normal")
            self.save_to_history(ip, name)
            self.update_history_list()
        except Exception as e:
            self.log(f"❌ Ошибка подключения к {ip}: {e}")
            messagebox.showerror("Ошибка", f"Не удалось подключиться: {e}")
            self.connection_socket = None

    def disconnect(self):
        if self.connection_socket:
            try:
                self.connection_socket.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.settimeout(0.3)
                sock.bind(("", BROADCAST_PORT))
                sock.sendto(b"TURNOFF_ROBOHAND", ('<broadcast>', BROADCAST_PORT))                
            except:
                pass
            self.connection_socket = None
        self.log("🔌 Отключено")
        self.status_label.configure(text="Отключено")
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")

    def send_angles_periodically(self):
        """Отправляет текущие углы каждые 50 мс, если подключено"""
        if self.connection_socket:
            try:
                angles = [int(self.angle_sliders[i].get()) for i in range(tcpMessageSize)]
                self.connection_socket.send(bytes(angles))
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.log("⚠️ Соединение потеряно")
                self.disconnect()
            except Exception as e:
                self.log(f"❌ Ошибка отправки: {e}")

        # Повтор через 50 мс
        self.after(50, self.send_angles_periodically)

    def save_to_history(self, ip, name):
        entry = {"ip": ip, "name": name, "time": time.time()}
        self.history = [h for h in self.history if h["ip"] != ip]
        self.history.insert(0, entry)
        self.history = self.history[:10]
        with open(HISTORY_FILE, "w", encoding="utf-8") as f:
            json.dump(self.history, f, indent=2, ensure_ascii=False)

    def load_history(self):
        if os.path.exists(HISTORY_FILE):
            try:
                with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                return [item for item in data if "ip" in item]
            except:
                pass
        return []

    def update_history_list(self):
        self.history_listbox.delete(0, tk.END)
        for item in self.history:
            dt = datetime.fromtimestamp(item["time"]).strftime("%d.%m %H:%M")
            self.history_listbox.insert(tk.END, f"{item['name']} — {item['ip']} ({dt})")


# === Запуск приложения ===
if __name__ == "__main__":
    app = RobotClientApp()
    app.mainloop()