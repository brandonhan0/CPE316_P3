from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import serial_plot   # imports fig, update, ser
import time




if __name__ == "__main__":
    ani = FuncAnimation(serial_plot.fig, serial_plot.update, interval=50, blit=True)
    plt.tight_layout()
    plt.show()
    serial_plot.ser.close()


