import csv
import matplotlib.pyplot as plt
import numpy as np

##### Fix broken files #####
# with open('foo.csv', newline='') as in_file:
#     with open('ffoo.csv', 'w', newline='') as out_file:
#         writer = csv.writer(out_file)
#         for row in csv.reader(in_file):
#             if row:
#                 writer.writerow(row)
                
# with open('compFilter.csv', newline='') as in_file:
#     with open('FixedcompFilter.csv', 'w', newline='') as out_file:
#         writer = csv.writer(out_file)
#         for row in csv.reader(in_file):
#             if row:
#                 writer.writerow(row)

##### Str TREKF #####
with open('TREKfoo.csv', newline='') as in_file:
    with open('TREKffoo.csv', 'w', newline='') as out_file:
        writer = csv.writer(out_file)
        for row in csv.reader(in_file):
            if row:
                writer.writerow(row)

data = csv.reader(open('TREKffoo.csv', 'rt'), delimiter=',')
r, p, y, fr, fp, fy, rr, rp, ry = [], [], [], [], [], [], [], [], []  

for row in data:
    r.append(round(float(row[0]),3))
    p.append(round(float(row[1]),3))
    y.append(round(float(row[2]),3))
    fr.append(round(float(row[3]),3))
    fp.append(round(float(row[4]),3))
    fy.append(round(float(row[5]),3))
    rr.append(round(float(row[6]),3)+180)
    rp.append(round(-float(row[7]),3)+1)
    ry.append(round(-float(row[8]),3)+70)


plt.plot(list(np.arange(len(ry))*0.01), rr, label='EKF Roll')
plt.plot(list(np.arange(len(y))*0.01), r, label='REKF Roll')
plt.plot(list(np.arange(len(fy))*0.01), fr, label='Filtered Roll')
ax = plt.gca()
#ax.set_ylim(bottom=-170, top=-75)
#ax.set_xbound(lower=3, upper=35)
plt.grid()
#plt.title("Robust Extended Kalman with Elliptic Bandstop Filter of Variable 4-6Hz Tremor Emulation")
#plt.title("Robust Extended Kalman with Bandstop Filter Output")
plt.title("Extended Kalman Filter Outputs During Robotic Jig Yaw-Pitch Motion")
plt.xlabel("Time (s)")
plt.ylabel("Rotation (°)")
#ax.axhline(y = ym, color = 'r', linestyle = '-', label='Mean Yaw') 
ax.legend()
plt.show()
##### End TREKF #####       

##### CSV to Graph #####
data = csv.reader(open('ffooREKF.csv', 'rt'), delimiter=',')
r, p, y = [], [], []

for row in data:
    r.append(round(float(row[0]),3))
    p.append(round(float(row[1]),3))
    y.append(round(float(row[2]),3))

rm = np.mean(r)
pm = np.mean(p)
ym = np.mean(y)

# plt.plot(list(np.arange(len(y))*0.01), y, label='Yaw')
# ax = plt.gca()
# #ax.set_ylim(bottom=-170, top=-75)
# #ax.set_xbound(lower=3, upper=35)
# plt.grid()
# plt.title("EK Filter Output of Variable Tremor Emulation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = ym, color = 'r', linestyle = '-', label='Mean Yaw') 
# ax.legend()
# plt.show()

# #r = [float(i)/max(r) for i in r]
# plt.plot(list(np.arange(len(r))*0.01), r, label='Roll')
# ax = plt.gca()
# #ax.set_ylim(bottom=-170, top=-75)
# ax.set_xbound(lower=5, upper=25)
# plt.grid()
# plt.title("Adapted EK Filter Output during stepper motor rotation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# #ax.axhline(y = rm, color = 'r', linestyle = '-', label='Mean Roll') 
# ax.legend()
# plt.show()

data = csv.reader(open('FixednormalEKF.csv', 'rt'), delimiter=',')
roll, pitch, yaw = [], [], []

for row in data:
    roll.append(round(float(row[0]),3))
    pitch.append(round(float(row[1]),3))
    yaw.append(round(float(row[2]),3))

rollmean = np.mean(roll)
pitchmean = np.mean(pitch)
yawmean = np.mean(yaw)

tremorEKFdata = csv.reader(open('FixedtremorEKF.csv', 'rt'), delimiter=',')
teroll, tepitch, teyaw = [], [], []

for row in tremorEKFdata:
    teroll.append(round(float(row[0]),3))
    tepitch.append(round(float(row[1]),3))
    teyaw.append(round(float(row[2]),3))

terollmean = np.mean(teroll)
tepitchmean = np.mean(tepitch)
teyawmean = np.mean(teyaw)

randTremorEKFdata = csv.reader(open('FixedrandTremorEKF.csv', 'rt'), delimiter=',')
rteroll, rtepitch, rteyaw = [], [], []

for row in randTremorEKFdata:
    rteroll.append(round(float(row[0]),3))
    rtepitch.append(round(float(row[1]),3))
    rteyaw.append(round(float(row[2]),3))

rterollmean = np.mean(rteroll)
rtepitchmean = np.mean(rtepitch)
rteyawmean = np.mean(rteyaw)

compdata = csv.reader(open('FixedcompFilter.csv', 'rt'), delimiter=',')
croll, cpitch, = [], []

for row in compdata:
    croll.append(round(float(row[0]),3))
    cpitch.append(round(float(row[1]),3))

crollmean = np.mean(croll)
cpitchmean = np.mean(cpitch)

# plt.plot(list(np.arange(len(roll))*0.01), roll, label='Roll')
# ax = plt.gca()
# ax.set_ylim(bottom=141, top=142)
# ax.set_xbound(lower=1, upper = 25)
# plt.grid()
# plt.title("Extended Kalman Filter Orentation Output")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = rollmean, color = 'r', linestyle = '-', label='Mean Roll') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(teroll))*0.01), teroll, label='Roll')
# ax = plt.gca()
# ax.set_ylim(bottom=132, top=140)
# ax.set_xbound(lower=1, upper=22)
# plt.grid()
# plt.title("Extended Kalman Filter Output of 6°, 2.5Hz Tremor Emulation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = terollmean, color = 'r', linestyle = '-', label='Mean Roll') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(roll))*0.01), pitch, label='Pitch')
# ax = plt.gca()
# ax.set_ylim(bottom=-35, top=-34)
# ax.set_xbound(lower=1, upper = 25)
# plt.grid()
# plt.title("Extended Kalman Filter Orentation Output")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = pitchmean, color = 'r', linestyle = '-', label='Mean Pitch') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(tepitch))*0.01), tepitch, label='Pitch')
# ax = plt.gca()
# ax.set_ylim(bottom=-35, top=-25)
# ax.set_xbound(lower=1, upper=22)
# plt.grid()
# plt.title("Extended Kalman Filter Output of 6°, 2.5Hz Tremor Emulation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = tepitchmean, color = 'r', linestyle = '-', label='Mean Pitch') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(rtepitch))*0.01), rtepitch, label='Pitch')
# ax = plt.gca()
# ax.set_ylim(bottom=-40, top=-28)
# ax.set_xbound(lower=1, upper=35)
# plt.grid()
# plt.title("Extended Kalman Filter Output of Variable Tremor Emulation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = rtepitchmean, color = 'r', linestyle = '-', label='Mean Pitch') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(cpitch))*0.01), cpitch, label='Pitch')
# ax = plt.gca()
# #ax.set_ylim(bottom=-102, top=-85)
# ax.set_xbound(lower=1, upper=35)
# plt.grid()
# plt.title("Complementary Filter Output of Variable Tremor Emulation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = cpitchmean, color = 'r', linestyle = '-', label='Mean Pitch') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(roll))*0.01), yaw, label='Yaw')
# ax = plt.gca()
# ax.set_ylim(bottom=-86, top=-85)
# ax.set_xbound(lower=1, upper = 25)
# plt.grid()
# plt.title("Extended Kalman Filter Orentation Output")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = yawmean, color = 'r', linestyle = '-', label='Mean Yaw') 
# ax.legend()
# plt.show()

# plt.plot(list(np.arange(len(teyaw))*0.01), teyaw, label='Yaw')
# ax = plt.gca()
# ax.set_ylim(bottom=-75, top=-63)
# ax.set_xbound(lower=1, upper=22)
# plt.grid()
# plt.title("Extended Kalman Filter Output of 6°, 2.5Hz Tremor Emulation")
# plt.xlabel("Time (s)")
# plt.ylabel("Rotation (°)")
# ax.axhline(y = teyawmean, color = 'r', linestyle = '-', label='Mean Yaw') 
# ax.legend()
# plt.show()