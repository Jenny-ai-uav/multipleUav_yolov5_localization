import openpyxl
wb=openpyxl.Workbook()
ws=wb.active
data=(
("UavAmount","depth","slobe","Xt","Yt","realXt","realYt","uav_x","uav_y","uav_z","folder","distanceOverUav","height","velocity","fov","framewidth"),
)

a=data[0][0]
for i in data:
   ws.append(i)
for i in data:
   ws.append(i)
wb.save(r'.\datasetGPS\output.xlsx')
wb.close()

# xl_path is destination xlsx spreadsheet
