""" CANSAT PICO RECEIVER node

Receives message requiring ACK over RFM69HCW SPI module - RECEIVER node
Must be tested togheter with test_emitter

See Tutorial : https://wiki.mchobby.be/index.php?title=ENG-CANSAT-PICO-RFM69HCW-TEST
See GitHub : https://github.com/mchobby/cansat-belgium-micropython/tree/main/test-rfm69

RFM69HCW breakout : https://shop.mchobby.be/product.php?id_product=1390
RFM69HCW breakout : https://www.adafruit.com/product/3071
"""
import  time
from machine import SPI, Pin
from rfm69 import RFM69
import json
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

FREQ           = 433.1
ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 100 # ID of this node

spi = SPI(0, polarity=0, phase=0, firstbit=SPI.MSB) # baudrate=50000,
nss = Pin( 5, Pin.OUT, value=True )
rst = Pin( 3, Pin.OUT, value=False )

rfm = RFM69( spi=spi, nss=nss, reset=rst )
rfm.frequency_mhz = FREQ

# Optionally set an encryption key (16 byte AES key). MUST match both
# on the transmitter and receiver (or be set to None to disable/the default).
rfm.encryption_key = ( ENCRYPTION_KEY )
rfm.node = NODE_ID # This instance is the node 123

vel_x, vel_y, vel_z = [0], [0], [0]
pos_x, pos_y, pos_z = [0], [0], [0]
ax_raw, ay_raw, az_raw = [0], [0], [0]
time_steps, temp_data, hum_data, press_data, co2_data, tvoc_data = [0], [], [], [], [], []

dt = 0.5

fig = plt.figure(figsize=(12,8))

# we maken een 3D plot voor snelheid
ax_vel_3d = fig.add_subplot(221, projection='3d')
ax_vel_3d.set_title("3D Velocity")
ax_vel_3d.set_xlabel("Velocity X (m/s)")
ax_vel_3d.set_ylabel("Velocity Y (m/s)")
ax_vel_3d.set_zlabel("Velocity Z (m/s)")

# we maken een 3D plot voor positie
ax_pos_3d = fig.add_subplot(222, projection='3d')
ax_pos_3d.set_title("3D Position")
ax_pos_3d.set_xlabel("Position X (m)")
ax_pos_3d.set_ylabel("Position Y (m)")
ax_pos_3d.set_zlabel("Position Z (m)")

ax_acc_3d = fig.add_subplot(223, projection='3d')
ax_acc_3d.set_title("3D Acceleration")
ax_acc_3d.set_xlabel("Acceleration X (m/s²)")
ax_acc_3d.set_ylabel("Acceleration Y (m/s²)")
ax_acc_3d.set_zlabel("Acceleration Z (m/s²)")

ax_env = fig.add_subplot(224)
ax_env.set_title("Environmental Data (Temp, Humidity, Pressure, CO2, TVOC)")
ax_env.set_xlabel("Time (s)")
ax_env.set_ylabel("Values")

#Kalman setup
F_k = np.array([[1, dt, dt**2/2],
                [0,  1,        dt],
                [0,  0,         1]])

H_k = np.array([[0, 0, 1]])  # Measure acceleration

# covariaties
Q_k = np.diag([1e-3, 1e-3, 1e-2])  # Tune for process noise
R_k = np.array([[0.1**2]])        # Accel measurement noise variance

# initiatie (aan te passen)
state_x = np.zeros((3, 1))
P_x = np.eye(3)
state_y = np.zeros((3, 1))
P_y = np.eye(3)
state_z = np.zeros((3, 1))
P_z = np.eye(3)

# Kalmanfunctie
def kalman_step(z_meas, state, P):
    # Predict
    x_pred = F_k @ state
    P_pred = F_k @ P @ F_k.T + Q_k
    # Innovation
    y = z_meas - H_k @ x_pred
    S = H_k @ P_pred @ H_k.T + R_k
    K = P_pred @ H_k.T @ np.linalg.inv(S)
    # Update
    x_upd = x_pred + K @ y
    P_upd = (np.eye(3) - K @ H_k) @ P_pred
    return x_upd, P_upd


#functie die de data integreert (voor grafiek zonder Kalmanfilter, als vergelijking, lijkt me leuk)
def integrate_data_without_filter(ax_raw, ay_raw, az_raw, dt, pos_x_prev, pos_y_prev, pos_z_prev):
    vel_x_non_filtered = np.cumsum(ax_raw) * dt
    vel_y_non_filtered = np.cumsum(ay_raw) * dt
    vel_z_non_filtered = np.cumsum(az_raw) * dt
    pos_x_non_filtered = pos_x_prev + np.cumsum(vel_x_non_filtered) * dt
    pos_y_non_filtered = pos_y_prev + np.cumsum(vel_y_non_filtered) * dt
    pos_z_non_filtered = pos_z_prev + np.cumsum(vel_z_non_filtered) * dt
    return pos_x_non_filtered, pos_y_non_filtered, pos_z_non_filtered, vel_x_non_filtered, vel_y_non_filtered, vel_z_non_filtered

update_time = time.time()
#nodig voor plot voorspelling zonder Kalman
def update_prediction(ax_raw, ay_raw, az_raw, dt, pos_x_prev, pos_y_prev, pos_z_prev, vel_x_prev, vel_y_prev, vel_z_prev):
    global update_time
    if time.time() - update_time > 10:
        update_time = time.time()
        pos_x_pred, pos_y_pred, pos_z_pred, vel_x_pred, vel_y_pred, vel_z_pred = integrate_data_without_filter(ax_raw, ay_raw, az_raw, dt, pos_x_prev, pos_y_prev, pos_z_prev)
        return pos_x_pred, pos_y_pred, pos_z_pred, vel_x_pred, vel_y_pred, vel_z_pred
    else:
        return pos_x_prev, pos_y_prev, pos_z_prev, vel_x_prev, vel_y_prev, vel_z_prev

print( 'Freq            :', rfm.frequency_mhz )
print( 'NODE            :', rfm.node )

print("Waiting for packets...")

data_punten = 60

def update_plot(frame):
	global time_steps, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, temp_data, hum_data, press_data, co2_data, tvoc_data, state_x, state_y, state_z, P_x, P_y, P_z, ax_filt_buf, ay_filt_buf, az_filt_buf
	packet = rfm.receive(with_ack=True)
	if packet is None:
		return ax_vel_3d, ax_pos_3d, ax_acc_3d, ax_env

	packet_text = str(packet, "ascii")
	data = json.loads(packet_text)

	# hier nemen we de data van de pico
	ax_raw = data['ax']
	ay_raw = data['ay']
	az_raw = data['az']

	#data Kalmanfilter
	state_x, P_x = kalman_step(np.array([[ax_raw]]), state_x, P_x)
	state_y, P_y = kalman_step(np.array([[ay_raw]]), state_y, P_y)
	state_z, P_z = kalman_step(np.array([[az_raw]]), state_z, P_z)

	#gefilterde data
	pos_x_filtered = state_x[0]
	vel_x_filtered = state_x[1]
	pos_y_filtered = state_y[0]
	vel_y_filtered = state_y[1]
	pos_z_filtered = state_z[0]
	vel_z_filtered = state_z[1]

#ruwe berekening
	pos_x_non_filtered, pos_y_non_filtered, pos_z_non_filtered, vel_x_non_filtered, vel_y_non_filtered, vel_z_non_filtered = integrate_data_without_filter(
		ax_raw, ay_raw, az_raw, dt, pos_x[-1], pos_y[-1], pos_z[-1])
#voorspellingsberekeningen
	pos_x_pred, pos_y_pred, pos_z_pred, vel_x_pred, vel_y_pred, vel_z_pred = update_prediction(
		ax_raw, ay_raw, az_raw, dt, pos_x[-1], pos_y[-1], pos_z[-1])

		# verandering voor plot
	pos_x.append(pos_x_filtered)
	pos_y.append(pos_y_filtered)
	pos_z.append(pos_z_filtered)

	vel_x.append(vel_x_filtered)
	vel_y.append(vel_y_filtered)
	vel_z.append(vel_z_filtered)
	temp_data.append(data.get("temp", 0))
	hum_data.append(data.get("humidity", 0))
	press_data.append(data.get("pressure", 0))
	co2_data.append(data.get("co2", 0))
	tvoc_data.append(data.get("tvoc", 0))


	# we schrijven het op een csv file
	with open("integrated_data.csv", "a") as f:
		writer = csv.writer(f)
		writer.writerow([time_steps[-1], ax_raw, ay_raw, az_raw, vel_x[-1], vel_y[-1], vel_z[-1], pos_x[-1], pos_y[-1], pos_z[-1]])

	time_steps.append(time_steps[-1] + dt if time_steps else 0)

	if len(temp_data) > data_punten:
		temp_data.pop(0)
		hum_data.pop(0)
		press_data.pop(0)
		co2_data.pop(0)
		tvoc_data.pop(0)

	# real-time update
	ax_vel_3d.cla()
	ax_vel_3d.set_title("3D Velocity Comparison")
	ax_vel_3d.set_xlabel("Velocity X (m/s)")
	ax_vel_3d.set_ylabel("Velocity Y (m/s)")
	ax_vel_3d.set_zlabel("Velocity Z (m/s)")

	ax_vel_3d.plot(vel_x_filtered, vel_y_filtered, vel_z_filtered, label="Filtered Velocity (3D)", color='r')
	ax_vel_3d.plot(vel_x_non_filtered, vel_y_non_filtered, vel_z_non_filtered, label="Unfiltered Velocity (3D)",
				   color='g')
	ax_vel_3d.plot(vel_x_pred, vel_y_pred, vel_z_pred, label="Predicted Velocity (3D)", color='b')

	# update positie
	ax_pos_3d.cla()
	ax_pos_3d.set_title("3D Position Comparison")
	ax_pos_3d.set_xlabel("Position X (m)")
	ax_pos_3d.set_ylabel("Position Y (m)")
	ax_pos_3d.set_zlabel("Position Z (m)")

	ax_pos_3d.plot(pos_x_filtered, pos_y_filtered, pos_z_filtered, label="Filtered Position (3D)", color='b')
	ax_pos_3d.plot(pos_x_non_filtered, pos_y_non_filtered, pos_z_non_filtered, label="Unfiltered Position (3D)",
				   color='y')
	ax_pos_3d.plot(pos_x_pred, pos_y_pred, pos_z_pred, label="Predicted Position (3D)", color='g')

	# Update acceleratie
	ax_acc_3d.cla()
	ax_acc_3d.set_title("3D Acceleration")
	ax_acc_3d.set_xlabel("Acceleration X (m/s²)")
	ax_acc_3d.set_ylabel("Acceleration Y (m/s²)")
	ax_acc_3d.set_zlabel("Acceleration Z (m/s²)")
	az_filt = state_z[2, 0]
	ay_filt = state_y[2, 0]
	ax_filt = state_x[2, 0]
	ax_acc_3d.plot([ax_raw], [ay_raw], [az_raw], label="Raw", color='g')
	ax_acc_3d.plot([ax_filt], [ay_filt], [az_filt], label="Filtered", color='r')

	# Update environmental data plot
	ax_env.cla()
	ax_env.set_title("Environmental Data (Temp, Humidity, Pressure, CO2, TVOC)")
	ax_env.set_xlabel("Time (s)")
	ax_env.set_ylabel("Values")

	ax_env.plot(time_steps[-data_punten:], temp_data, label="Temp (°C)")
	ax_env.plot(time_steps[-data_punten:], hum_data, label="Humidity (%)")
	ax_env.plot(time_steps[-data_punten:], press_data, label="Pressure (hPa)")
	ax_env.plot(time_steps[-data_punten:], co2_data, label="CO2 (ppm)")
	ax_env.plot(time_steps[-data_punten:], tvoc_data, label="TVOC (ppb)")
	ax_env.legend()

	return ax_vel_3d, ax_pos_3d, ax_acc_3d, ax_env

# Real-time animation update

ani = FuncAnimation(fig, update_plot, blit=False, interval=500)
plt.tight_layout()
plt.show()