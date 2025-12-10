import serial
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import glob
from openai import OpenAI


def sht3x_convert_temperature_c(rawT: int) -> float:
    """Convert raw 16-bit SHT3x temperature to °C."""
    return -45.0 + 175.0 * (rawT / 65535.0)


def sht3x_convert_rh(rawRH: int) -> float:
    """Convert raw 16-bit SHT3x humidity to %RH."""
    return 100.0 * (rawRH / 65535.0)

def moisture_percent(rawM: int, DRY=3900, WET=1000) -> float:
    """
    Convert raw ADC (0-4095) to soil moisture % using calibration values.
    DRY = ADC value in dry soil
    WET = ADC value in fully wet soil
    """
    print(rawM) # rawM = (3900 dry - 1000 wet)
    rawM_new = rawM
    raw_percent =  (rawM / 4100) # should give decimal less than 1
    print(f"raw_percent:{raw_percent}")
    wet_percent = (1 - raw_percent) # 0.04 dry, 0.75 wet
    print(f"wet_percent:{wet_percent}")
    wet_percent = (wet_percent * 120.0)
    print(f"scaled wet_percent:{wet_percent}")
    return wet_percent

# ================== USER SETTINGS ==================
candidates = glob.glob("/dev/tty.usbmodem*")
if not candidates:
    raise Exception("No STM32 detected!")
PORT = candidates[0]
BAUDRATE = 115200    # <-- change to match your UART baud rate
MAX_POINTS = 200     # number of data points to show in the plot
FRAME_SIZE = 8
MAX_POINTS = 200
# ===================================================



client = OpenAI(api_key="")
ai_counter = 0
def ai_response(temp, humid, moist):
    prompt = (
        f"Temperature: {temp:.2f} C\n"
        f"Humidity: {humid:.2f} %\n"
        f"Soil moisture: {moist:.2f} %\n\n"
        "Give a concise summary and any advice for plant care."
    )
    resp = client.responses.create(
        model="gpt-5-nano",
        instructions="Summarize the plant health and suggest care.",
        input=prompt,
    )
    print("\n=== AI PLANT SUMMARY ===")
    print(resp.output_text)
    print("========================\n")


# ---------------- Serial Setup ----------------

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)  # allow STM32 to reset USB CDC

# ---------------- Data Buffers ----------------

temps  = deque(maxlen=MAX_POINTS)
humids = deque(maxlen=MAX_POINTS)
moists = deque(maxlen=MAX_POINTS)

# ---------------- Plot Setup ----------------

plt.style.use("default")
fig, (axT, axH, axM) = plt.subplots(3, 1, sharex=True, figsize=(8, 6))

lineT, = axT.plot([], [], lw=1)
lineH, = axH.plot([], [], lw=1)
lineM, = axM.plot([], [], lw=1)

axT.set_ylabel("Temp (°C)")
axH.set_ylabel("Humidity (%)")
axM.set_ylabel("Moisture (%)")
axM.set_xlabel("Sample #")

axT.grid(True)
axH.grid(True)
axM.grid(True)

# ---------------- Animation Update Function ----------------
def update(frame):
    global ai_counter
    raw = ser.read(FRAME_SIZE)
    if len(raw) != FRAME_SIZE:
        return lineT, lineH, lineM

        # ----- Unpack 8-byte frame -----
    adc_raw = raw[0] | (raw[1] << 8)

    t_raw  = (raw[2] << 8) | raw[3]
    rh_raw = (raw[5] << 8) | raw[6]

    # ----- Convert to physical values -----
    temp_c = sht3x_convert_temperature_c(t_raw)
    rh_pct = sht3x_convert_rh(rh_raw)
    moist_pct = moisture_percent(adc_raw)

    # Append values to buffers
    temps.append(temp_c)
    humids.append(rh_pct)
    moists.append(moist_pct)

    # X-axis indexes
    x = range(len(temps))

    # Update lines
    lineT.set_data(x, temps)
    lineH.set_data(x, humids)
    lineM.set_data(x, moists)

    # Rescale Y axes automatically
    if temps:
        axT.set_ylim(min(temps)-1, max(temps)+1)
    if humids:
        axH.set_ylim(0, 100)
    if moists:
        #axM.set_ylim(min(moists)-5, max(moists)+5)
        axM.set_ylim(0, 100)

    axT.set_xlim(0, MAX_POINTS)

    # ai_counter += 1
    # if ai_counter % 200 == 0:
    #     try:
    #         ai_response(temp_c, rh_pct, moist_pct)
    #     except Exception as e:
    #         print("AI error:", e)

    return lineT, lineH, lineM



