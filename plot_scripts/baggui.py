import tkinter as tk
from tkinter import filedialog, messagebox
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class BagPlotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bag Plot Viewer")
        self.df = None
        self.canvas = None

        # Frame for controls
        control_frame = tk.Frame(root)
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        # File Load Button
        self.load_button = tk.Button(control_frame, text="Load CSV", command=self.load_file)
        self.load_button.pack(side=tk.LEFT, padx=5)

        # X Column Dropdown
        self.x_dropdown = tk.StringVar()
        self.x_menu = tk.OptionMenu(control_frame, self.x_dropdown, '')
        self.x_menu.pack(side=tk.LEFT, padx=5)

        # Y Column Dropdown
        self.y_dropdown = tk.StringVar()
        self.y_menu = tk.OptionMenu(control_frame, self.y_dropdown, '')
        self.y_menu.pack(side=tk.LEFT, padx=5)

        # Plot Button
        self.plot_button = tk.Button(control_frame, text="Plot", command=self.plot, state=tk.DISABLED)
        self.plot_button.pack(side=tk.LEFT, padx=5)

        # Frame for Matplotlib Figure
        self.plot_frame = tk.Frame(root)
        self.plot_frame.pack(fill=tk.BOTH, expand=True)

    def load_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not file_path:
            return

        try:
            self.df = pd.read_csv(file_path)
            self.populate_column_menus()
            self.plot_button.config(state=tk.NORMAL)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file:\n{e}")

    def populate_column_menus(self):
        if self.df is None or self.df.empty:
            return

        columns = list(self.df.columns)

        # Update X menu
        menu = self.x_menu['menu']
        menu.delete(0, 'end')
        for col in columns:
            menu.add_command(label=col, command=lambda value=col: self.x_dropdown.set(value))
        self.x_dropdown.set(columns[0])

        # Update Y menu
        menu = self.y_menu['menu']
        menu.delete(0, 'end')
        for col in columns:
            menu.add_command(label=col, command=lambda value=col: self.y_dropdown.set(value))
        self.y_dropdown.set(columns[1] if len(columns) > 1 else columns[0])

    def plot(self):
        if self.df is None:
            return

        x_col = self.x_dropdown.get()
        y_col = self.y_dropdown.get()

        try:
            df_plot = self.df[[x_col, y_col]].dropna()

            fig, ax = plt.subplots(figsize=(8, 5))
            ax.plot(df_plot[x_col], df_plot[y_col])
            ax.set_title("Bag Plot Example")
            ax.set_xlabel(x_col)
            ax.set_ylabel(y_col)

            if self.canvas:
                self.canvas.get_tk_widget().destroy()

            self.canvas = FigureCanvasTkAgg(fig, master=self.plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        except Exception as e:
            messagebox.showerror("Plot Error", f"Failed to plot:\n{e}")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("900x600")
    app = BagPlotApp(root)
    root.mainloop()

