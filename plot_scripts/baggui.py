import tkinter as tk
from tkinter import filedialog, messagebox
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import sys


class BagPlotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bag Plot Viewer")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.df = None
        self.canvas = None

        control_frame = tk.Frame(root)
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        self.load_button = tk.Button(control_frame, text="Load CSV", command=self.load_file)
        self.load_button.pack(side=tk.LEFT, padx=5)

        # Dropdowns
        self.x_dropdown = tk.StringVar()
        self.x_menu = tk.OptionMenu(control_frame, self.x_dropdown, '')
        self.x_menu.pack(side=tk.LEFT, padx=5)

        self.y1_dropdown = tk.StringVar()
        self.y1_menu = tk.OptionMenu(control_frame, self.y1_dropdown, '')
        self.y1_menu.pack(side=tk.LEFT, padx=5)

        self.y2_dropdown = tk.StringVar()
        self.y2_menu = tk.OptionMenu(control_frame, self.y2_dropdown, '')
        self.y2_menu.pack(side=tk.LEFT, padx=5)

        # Units
        self.x_unit = tk.Entry(control_frame, width=6)
        self.x_unit.insert(0, "unit")
        self.x_unit.pack(side=tk.LEFT, padx=2)

        self.y1_unit = tk.Entry(control_frame, width=6)
        self.y1_unit.insert(0, "unit")
        self.y1_unit.pack(side=tk.LEFT, padx=2)

        self.y2_unit = tk.Entry(control_frame, width=6)
        self.y2_unit.insert(0, "unit")
        self.y2_unit.pack(side=tk.LEFT, padx=2)

        # Trimming
        tk.Label(control_frame, text="Trim First").pack(side=tk.LEFT, padx=(10, 2))
        self.trim_start = tk.Spinbox(control_frame, from_=0, to=10000, width=5)
        self.trim_start.pack(side=tk.LEFT, padx=2)

        tk.Label(control_frame, text="Trim Last").pack(side=tk.LEFT, padx=(10, 2))
        self.trim_end = tk.Spinbox(control_frame, from_=0, to=10000, width=5)
        self.trim_end.pack(side=tk.LEFT, padx=2)

        self.plot_button = tk.Button(control_frame, text="Plot", command=self.plot, state=tk.DISABLED)
        self.plot_button.pack(side=tk.LEFT, padx=10)

        # Plot area
        self.plot_frame = tk.Frame(root)
        self.plot_frame.pack(fill=tk.BOTH, expand=True)

    def on_closing(self):
        self.root.destroy()
        sys.exit(0)

    def load_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not file_path:
            return

        try:
            self.df = pd.read_csv(file_path)
            self.populate_column_menus()
            self.plot_button.config(state=tk.NORMAL)
            self.df["time"] = (self.df["time"] - self.df["time"].iloc[0]) / 1e9
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file:\n{e}")

    def populate_column_menus(self):
        if self.df is None or self.df.empty:
            return

        columns = list(self.df.columns)

        # X menu
        menu = self.x_menu['menu']
        menu.delete(0, 'end')
        for col in columns:
            menu.add_command(label=col, command=lambda value=col: self.x_dropdown.set(value))
        self.x_dropdown.set(columns[0])

        # Y1 menu
        menu = self.y1_menu['menu']
        menu.delete(0, 'end')
        for col in columns:
            menu.add_command(label=col, command=lambda value=col: self.y1_dropdown.set(value))
        self.y1_dropdown.set(columns[1] if len(columns) > 1 else columns[0])

        # Y2 menu (optional)
        menu = self.y2_menu['menu']
        menu.delete(0, 'end')
        for col in columns:
            menu.add_command(label=col, command=lambda value=col: self.y2_dropdown.set(value))
        menu.add_command(label="None", command=lambda: self.y2_dropdown.set(""))
        self.y2_dropdown.set("")

    def plot(self):
        if self.df is None:
            return

        x_col = self.x_dropdown.get()
        y1_col = self.y1_dropdown.get()
        y2_col = self.y2_dropdown.get()

        try:
            columns_to_use = [x_col, y1_col]
            if y2_col and y2_col in self.df.columns:
                columns_to_use.append(y2_col)

            df_plot = self.df[columns_to_use].copy()

            # Interpolate only numeric columns, keeping index aligned
            df_plot = df_plot.interpolate(method='linear', limit_direction='both', axis=0)

            trim_start = int(self.trim_start.get())
            trim_end = int(self.trim_end.get())
            if trim_start + trim_end >= len(df_plot):
                messagebox.showwarning("Warning", "Trim values remove all data.")
                return

            if trim_end > 0:
                df_plot = df_plot.iloc[trim_start:-trim_end]
            else:
                df_plot = df_plot.iloc[trim_start:]

            fig, ax1 = plt.subplots(figsize=(9, 5))

            # Primary Y1
            ax1.plot(df_plot[x_col], df_plot[y1_col], color='tab:blue', label=y1_col)

            if self.x_unit.get()=="":
                xlabel = f"{x_col}".split("/")[-1]
                ax1.set_xlabel(xlabel)
            else:
                xlabel = f"{x_col}".split("/")[-1]
                ax1.set_xlabel(f"{xlabel} ({self.x_unit.get()})")


            if self.y1_unit.get()=="":
                y1label = f"{y1_col}".split("/")[-1]
                ax1.set_ylabel(y1label)
            else:
                y1label = f"{y1_col}".split("/")[-1]
                ax1.set_ylabel(f"{y1label} ({self.y1_unit.get()})")

            ax1.tick_params(axis='y', labelcolor='tab:blue')

            # Secondary Y2 (if set)
            y2 = y2_col if y2_col in df_plot.columns else None
            if y2:
                ax2 = ax1.twinx()
                ax2.plot(df_plot[x_col], df_plot[y2], color='tab:orange', label=y2)

                if self.y2_unit.get()=="":
                    y2label = f"{y2_col}".split()[-1]
                    ax1.set_ylabel(y2label)
                else:
                    y2label = f"{y2_col}".split()[-1]
                    ax1.set_ylabel(f"{y2label} ({self.y2_unit.get()})")

                ax2.set_ylabel(f"{y2} ({self.y2_unit.get()})", color='tab:orange')
                ax2.tick_params(axis='y', labelcolor='tab:orange')

            fig.tight_layout()

            if self.canvas:
                self.canvas.get_tk_widget().destroy()

            self.canvas = FigureCanvasTkAgg(fig, master=self.plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        except Exception as e:
            messagebox.showerror("Plot Error", f"Failed to plot:\n{e}")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1200x700")
    app = BagPlotApp(root)
    root.mainloop()

