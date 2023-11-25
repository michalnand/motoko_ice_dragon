import tkinter as tk
from tkinter import ttk

class Application(ttk.Frame):
    def __init__(self, master=None):
        super().__init__(master, style="Dark.TFrame")
        self.master = master
        self.pack(expand = True, fill = tk.BOTH)
        self.create_widgets()

    def create_widgets(self):
        self.label = ttk.Label(self, text="Hello, dark world!")
        self.label.pack(padx=10, pady=10)

        self.button = ttk.Button(self, text="Click me!", 
                                 command=self.on_click)
        self.button.pack(padx=10, pady=10)

        self.entry = ttk.Entry(self, width=20)
        self.entry.pack(padx=10, pady=10)

        self.checkbutton_var = tk.BooleanVar(value=True)
        self.checkbutton = ttk.Checkbutton(self, text="Check me!", 
                                           variable=self.checkbutton_var)
        self.checkbutton.pack(padx=10, pady=10)

        self.combobox = ttk.Combobox(self, values=["Option 1", "Option 2", "Option 3"])
        self.combobox.pack(padx=10, pady=10)

    def on_click(self):
        self.label.configure(text="Button clicked!")


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Dark Mode Tkinter Application")
    root.geometry("300x300")

    app = Application(master=root)
    app.mainloop()