import matplotlib.pyplot as plt

filename = "/home/agathe/Documents/A2/Stage/Projet/smart-water-quality-monitoring/RaspberryBleRos/catkin_ws/src/bluetooth_sensors/src/kalman_graph.txt"

ph_raw, ph_filt, ec_raw, ec_filt = [], [], [], []

with open(filename, 'r') as f:
  for line in f:
    points = line.replace("\n","").split(";")
    if float(points[0]) !=-1:
        ph_raw.append(points[0])
        ph_filt.append(points[1])
        ec_raw.append(points[2])
        ec_filt.append(points[3])
    

abscissa = np.arange(0, len(ph_raw))
plt.subplot(2, 1, 1)
plt.plot(abscissa, np.array(ph_raw), 'c', label="measured")
plt.plot(abscissa, np.array(ph_filt),'r',label="estimed")
plt.title('Kalman filter applied to sensors as a function of time')
plt.ylabel('pH')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(abscissa, np.array(ec_raw), 'c', label="measured")
plt.plot(abscissa, np.array(ec_filt),'r',label="estimed")
plt.ylabel('Condictivity (mS/cm)')

plt.legend()
plt.show()

f.close()