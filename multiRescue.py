# 匯入需要的模組
import time
import airsim
import cv2
import detectJ as detect  #可以看是否有精進辨識速度或辨識率的方式
import os
import numpy as np
import json
import math

from multiprocessing import Pool,Queue
from pathPlan import evenTrajectory,oddTrajectory,dividuleTrajectory
#將實驗記錄寫入excel
import openpyxl
#搜索基準點#實驗參數設定

#搜索基準點
alldata=[]

def drone_init(droneName):  #無人機初始化-初始位置自動記錄
    client=airsim.MultirotorClient()  #與無人機連線
    client.armDisarm(True,vehicle_name=droneName)   #初始化
    #讀取模擬環境設定檔，取得無人機初始位置
    AirsimSettingPath='C:\\Users\\a0930\\Documents\\AirSim\\settings.json'
    with open(AirsimSettingPath) as f:
        conf=json.load(f)
    droneX_init=(conf['Vehicles'][droneName]['X'])
    droneY_init=(conf['Vehicles'][droneName]['Y'])
    return[droneName,droneX_init,droneY_init]   #回傳無人機名稱、初始x及y座標

def getTargetPos(client,objName,number):    #取得所有目標物的精確位置
    posDic={}
    for i in range(1,number+1):
        pos=client.simGetObjectPose(object_name='{0}{1}'.format(objName,i)).position
        posDic['{0}:{1}'.format(objName,i)]=(pos.x_val,pos.y_val,pos.z_val)
    return (posDic)

#規劃無人機飛行路線
#以基準點劃分出搜索範圍
def trajectory(altitude,velocity,stripwidth,Radius,droneNumber):
    path = []
    trajectLog = []
    distance = 0
    print("start calculate the survey path")
    # AirSim uses NED coordinates so negative axis is up.
    z = -altitude
    iteration = 1
    maxLOOP = Radius / (3 * stripwidth)
    if droneNumber % 2 == 0:  # 若為雙數無人機，向右飛行
        startX = cx - Radius
        startY = cy + stripwidth
        # 基準點由中心先向左右各stripwidth後開始描繪
        startposition = airsim.Vector3r(startX, startY, z)
        path.append(startposition)
        trajectLog.append([startX, startY, z])
        while iteration < maxLOOP:
            iteration += 1
            startX += (Radius * 2)
            startY = startY
            distance += (Radius * 2)
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
            startX = startX
            startY += stripwidth
            distance += stripwidth
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
            startX -= (Radius * 2)
            startY = startY
            distance += (Radius * 2)
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
            startX = startX
            startY += stripwidth
            distance += stripwidth
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
    else:  # 若為單數無人機，向左飛行
        startX = cx - Radius
        startY = cy - stripwidth
        # 基準點由中心先向左右各stripwidth後開始描繪
        startposition = airsim.Vector3r(startX, startY, z)
        path.append(startposition)
        trajectLog.append([startX, startY, z])
        while iteration < maxLOOP:
            iteration += 1
            startX += (Radius * 2)
            startY = startY
            distance += (Radius * 2)
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
            startX = startX
            startY -= stripwidth
            distance += stripwidth
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
            startX -= (Radius * 2)
            startY = startY
            distance += (Radius * 2)
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
            startX = startX
            startY -= stripwidth
            distance +=stripwidth
            path.append(airsim.Vector3r(startX, startY, z))
            trajectLog.append([startX, startY, z])
    print("trajectory is \n{0}".format(trajectLog))
    print("starting survey, estimated distance is " + str(distance))
    # 飛行時間:距離/速度
    trip_time = distance / velocity
    print("estimated survey time is " + str(trip_time))
    return [path,trip_time]

def flytoStartpoint(startpoint,velocity,droneName):
    startpoint=startpoint[0]
    droneClient = airsim.MultirotorClient()
    droneClient.enableApiControl(True, vehicle_name=droneName)
    droneClient.takeoffAsync(vehicle_name=droneName)
    time.sleep(1)
    # 先飛到起始點
    droneClient.moveToPositionAsync(startpoint.x_val, startpoint.y_val, startpoint.z_val,5,vehicle_name=droneName).join()
    print("{0} reach to the start point".format(droneName))
    return "start"

def startSurey(route,velocity,droneName,triptime):#從起點開始進行既定搜索路線飛行
    surveyClient=airsim.MultirotorClient()
    surveyClient.confirmConnection()
    surveyClient.enableApiControl(True,vehicle_name=droneName)
    surveyClient.takeoffAsync(vehicle_name=droneName)
    time.sleep(1)
    surveyClient.moveOnPathAsync(route, velocity, triptime, airsim.DrivetrainType.ForwardOnly,
                                 airsim.YawMode(False, 0), vehicle_name=droneName)
def getPostion(droneName):#取得無人機飛行的即時座標
    client = airsim.MultirotorClient()
    while True:
        position = client.getMultirotorState(vehicle_name=droneName).kinematics_estimated.position
        print('[{0},{1},{2}-{3}]'.format(round(position.x_val, 2), round(position.y_val, 2), round(position.z_val, 2),
                                       droneName))
        time.sleep(1)

def parser_detectResult(detectList):#第一次偵測目標物並回傳,閥值0.3
    arr=[]
    for item in detectList:
        if 'person' in item[0] :
            content=item[0].split(' ')
            if float(content[1])>0.4:
                arr.append((item[1],item[2]))
    return arr

def parser_detectResult2(detectList):#二次偵測目標物並回傳,閥值0.2
    arr=[]
    for item in detectList:
        if 'person' in item[0] :
            content=item[0].split(' ')
            if float(content[1])>0.2:
                arr.append((item[1],item[2]))
    return arr

#取得偵測圖像後進行距離與目標物位置計算
def CalcTheta(frameWidth,Hfov,posx1,posx2):
    posx=(posx1+posx2)/2#算出目標物圖片中心點
    if posx>frameWidth/2:#若再中心點(無人機)右側
        Theta=((posx-frameWidth/2))*Hfov/frameWidth#角度為正值
    else:#若再中心點(無人機)左側
        Theta=((frameWidth/2)-posx)*Hfov/frameWidth
        Theta=-Theta#角度為正值
    return Theta

def caculateTargetPositionX(thetaResult,depth, CameraPos):
    cosinTheta = math.cos(math.radians(thetaResult))
    slobesize = ((depth * 100 / 255) + CameraPos) / cosinTheta
    print('slobesize:{0}'.format(slobesize))
    return slobesize

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians\

def distance2target(targetHeight,focalLength,HeightPixel):  #計算後得出focalLenght=3.6
    print('Pixel From Height:{0}'.format(HeightPixel))
    distance=(targetHeight*focalLength)/HeightPixel  #unit is cm
    return distance

def realDistance(uavX,uavY,targetX,targetY):
    x_2=(targetX-uavX)**2
    y_2=(targetY-uavY)**2
    d=(x_2+y_2)**0.5
    return d

def rotate(x,y,angle):
    xr=x*math.cos(math.radians(angle))-y*math.sin(math.radians(angle))
    yr=x*math.sin(math.radians(angle))+y*math.cos(math.radians(angle))
    return [xr,yr]

def targetDetect(cameraName, ImageType, drone, Fov,Folderpath,FrameWidth,detectList):
    tc = airsim.MultirotorClient()
    while True:
        rawImage = tc.simGetImage(cameraName, airsim.ImageType.Scene, vehicle_name=drone)
        frame = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_COLOR)
        detect_result = detect.main(frame)
        result = parser_detectResult(detect_result)
        returnList=[]
        if (result == []):
            continue
        else:
            tc.simPause(True)
            while True:
                responses = tc.simGetImages(
                    [airsim.ImageRequest(0, ImageType, pixels_as_float=False, compress=False),
                     airsim.ImageRequest(cameraName, airsim.ImageType.DepthPlanar, True, False)], vehicle_name=drone)
                img_1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                frame = img_1d.reshape(responses[0].height, responses[0].width, 3)
                detect_result = detect.main(frame)
                result = parser_detectResult2(detect_result)
                if (result == []):
                    tc.simPause(False)
                    continue  # original is break
                else:
                    returnList.append(True)  # 加入返回值
                    returnList.append(drone)  #加入返回值
                    uav1_pose = tc.getMultirotorState(vehicle_name=drone).kinematics_estimated.position
                    uav1_orientation = tc.getMultirotorState(vehicle_name=drone).kinematics_estimated.orientation
                    print("Uav{0}_pose:{1},{2},{3}".format(drone, uav1_pose.x_val, uav1_pose.y_val, uav1_pose.z_val))
                    print("Uav{0}_orientation:{1},{2},{3},{4}".format(drone, uav1_orientation.x_val,uav1_orientation.y_val, uav1_orientation.z_val, uav1_orientation.w_val))
                    returnList.append([uav1_pose.x_val, uav1_pose.y_val, uav1_pose.z_val])#加入返回值
                    t = time.time()
                    cv2.rectangle(frame, result[0][0], result[0][1], (255, 0, 255), thickness=1, lineType=cv2.LINE_AA)
                    cv2.imshow('{0}:scene'.format(drone), frame)  # show original image and detect result
                    cv2.waitKey(1)  # 1 millise
                    cv2.imwrite(r"{0}\{1}-{2}-firstDetected.png".format(Folderpath, t,drone), frame)
                    img_depth_planar = airsim.list_to_2d_float_array(responses[1].image_data_float, responses[1].width,
                                                                     responses[1].height)
                    cv2.imwrite(r"{0}\{1}-{2}-depth1.png".format(Folderpath, t,drone), img_depth_planar)
                    img_depth_planar = img_depth_planar.reshape(responses[1].height, responses[1].width, 1)
                    cv2.imwrite(r"{0}\{1}-{2}-depth2.png".format(Folderpath, t,drone), img_depth_planar)
                    depth_8bit_lerped = np.interp(img_depth_planar, (0, 100), (0, 255))
                    img_depth_planar = depth_8bit_lerped.astype('uint8')
                    tc.simPause(False)

                    w = (result[0][0][0] + result[0][1][0]) // 2
                    h = (result[0][0][1] + result[0][1][1]) // 2
                    depth = int(img_depth_planar[h][w])

                    camera_info = tc.simGetCameraInfo(camera_name=cameraName, vehicle_name=drone)

                    cv2.destroyAllWindows()
                    roll, pitch, yaw = euler_from_quaternion(uav1_orientation.x_val, uav1_orientation.y_val,
                                                             uav1_orientation.z_val, uav1_orientation.w_val)
                    currentYaw = yaw * 180 / math.pi
                    pixeltheta = (CalcTheta(FrameWidth, Fov, result[0][0][0], result[0][1][0]))
                    alltheta = currentYaw + pixeltheta

                    X_init = drone_init(drone)[1]
                    Y_init = drone_init(drone)[2]
                    CameraPos = math.sqrt((math.pow(uav1_pose.x_val - camera_info.pose.position.x_val, 2) + (
                        math.pow(uav1_pose.y_val - camera_info.pose.position.y_val, 2))))
                    # Distance from Depth image---255-100
                    slobe = caculateTargetPositionX(pixeltheta, depth, CameraPos)
                    returnList.append(slobe)#distance from target
                    dx = slobe * math.cos(math.radians(alltheta))  # 原本的正常視角-dx
                    dy = slobe * math.sin(math.radians(alltheta))  # 原本的正常視角-dy
                    position=rotate(dx,dy,alltheta)
                    print(position)
                    x_new=position[0]+ (uav1_pose.x_val) + X_init
                    y_new=position[1]+ (uav1_pose.x_val) + X_init
                    x = dx + (uav1_pose.x_val) + X_init
                    y = dy + (uav1_pose.y_val) + Y_init
                    returnList.append([x,y,uav1_pose.z_val])#預測目標物位置
                    print('Depth-image target evaluated position:{0} , {1}'.format(x, y))
                    detectList[2] = 'firstDetect_{0}'.format(drone)
                    detectList[12] = uav1_pose.x_val
                    detectList[13] = uav1_pose.y_val
                    detectList[14] = uav1_pose.z_val
                    detectList[15] = depth
                    detectList[16] = slobe
                    detectList[4] = x
                    detectList[5] = y
                    detectList[0] = t
                    returnList.append(detectList)
                    Mypath = r"{0}\{1}_{2}-firstDetected.txt".format(Folderpath, t,drone)
                    with open(Mypath, 'w') as f:
                        f.write('uav-pos:{0},{1},{2}\n'.format(uav1_pose.x_val, uav1_pose.y_val, uav1_pose.z_val))
                        f.write('depth:{0}\n'.format(depth))
                        f.write('slobe:{0}\n'.format(slobe))
                        f.write('Depth-image evaluation target position:{0}\n'.format([x, y]))
                        f.write('saveFolder:{0}\n'.format(t))
                        f.write('new xr yr:{0}\n'.format(position))
                        f.write('new position:{0},{1}\n'.format(x_new,y_new))
                        break
            result.clear()
            queue.put(returnList)
        return returnList
def targetDetect_search(cameraName, ImageType, drone,Fov,Folderpath,FrameWidth,detectList,ws):
    tc = airsim.MultirotorClient()
    responses = tc.simGetImages(
        [airsim.ImageRequest(cameraName, ImageType, pixels_as_float=False, compress=False),
         airsim.ImageRequest(cameraName, airsim.ImageType.DepthPlanar, True, False)], vehicle_name=drone)
    img_1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    frame = img_1d.reshape(responses[0].height, responses[0].width, 3)
    detect_result = detect.main(frame)
    result = parser_detectResult2(detect_result)
    if (result == []):
        print('no person detect until now')
    else:
        droneNumber = int(drone.replace('Drone', ""))
        uav1_pose = tc.getMultirotorState(vehicle_name=drone).kinematics_estimated.position
        uav1_orientation = tc.getMultirotorState(vehicle_name=drone).kinematics_estimated.orientation
        print("Uav{0}_pose:{1},{2},{3}".format(drone, uav1_pose.x_val, uav1_pose.y_val, uav1_pose.z_val))
        print("Uav{0}_orientation:{1},{2},{3},{4}".format(drone, uav1_orientation.x_val,
                                                          uav1_orientation.y_val,
                                                          uav1_orientation.z_val, uav1_orientation.w_val))
        t = time.time()
        cv2.rectangle(frame, result[0][0], result[0][1], (255, 0, 255), thickness=1, lineType=cv2.LINE_AA)
        cv2.imshow('{0}:scene'.format(drone), frame)  # show original image and detect result
        cv2.waitKey(1)  # 1 millise
        cv2.imwrite(r"{0}\{1}-{2}-scene.png".format(Folderpath, t, drone), frame)

        img_depth_planar = airsim.list_to_2d_float_array(responses[1].image_data_float, responses[1].width,
                                                         responses[1].height)
        img_depth_planar = img_depth_planar.reshape(responses[1].height, responses[1].width, 1)
        depth_8bit_lerped = np.interp(img_depth_planar, (0, 100), (0, 255))
        img_depth_planar = depth_8bit_lerped.astype('uint8')
        tc.simPause(False)
        w = (result[0][0][0] + result[0][1][0]) // 2
        h = (result[0][0][1] + result[0][1][1]) // 2

        depth = int(img_depth_planar[h][w])

        camera_info = tc.simGetCameraInfo(camera_name=cameraName, vehicle_name=drone)
        cv2.destroyAllWindows()
        roll, pitch, yaw = euler_from_quaternion(uav1_orientation.x_val, uav1_orientation.y_val,
                                                 uav1_orientation.z_val, uav1_orientation.w_val)
        currentYaw = yaw * 180 / math.pi
        pixeltheta = (CalcTheta(FrameWidth, Fov, result[0][0][0], result[0][1][0]))
        alltheta = currentYaw + pixeltheta

        X_init = drone_init(drone)[1]
        Y_init = drone_init(drone)[2]
        CameraPos = math.sqrt((math.pow(uav1_pose.x_val - camera_info.pose.position.x_val, 2) + (
            math.pow(uav1_pose.y_val - camera_info.pose.position.y_val, 2))))
        slobe = caculateTargetPositionX(pixeltheta, depth, CameraPos)
        dx = slobe * math.cos(math.radians(alltheta))  # 原本的正常視角-dx
        dy = slobe * math.sin(math.radians(alltheta))  # 原本的正常視角-dy

        position = rotate(dx, dy, alltheta)
        print(position)
        x_new = position[0] + (uav1_pose.x_val) + X_init
        y_new = position[1] + (uav1_pose.x_val) + X_init

        x = dx + (uav1_pose.x_val) + X_init
        y = dy + (uav1_pose.y_val) + Y_init
        print('Depth-image target evaluated position:{0} , {1}'.format(x, y))
        Mypath = r"{0}\{1}_{2}.txt".format(Folderpath, t, drone)
        detectList[2] = drone
        detectList[12] = uav1_pose.x_val
        detectList[13] = uav1_pose.y_val
        detectList[14] = uav1_pose.z_val
        detectList[15] = depth
        detectList[16] = slobe
        detectList[4] = x
        detectList[5] = y
        detectList[0] = t
        ws.append(detectList)
        with open(Mypath, 'w') as f:
            f.write('uav-pos:{0},{1},{2}\n'.format(uav1_pose.x_val, uav1_pose.y_val, uav1_pose.z_val))
            f.write('depth:{0}\n'.format(depth))
            f.write('slobe:{0}\n'.format(slobe))
            f.write('Depth-image evaluation target position:{0}\n'.format([x, y]))
            f.write('saveFolder:{0}\n'.format(t))
            f.write('new xr yr:{0}\n'.format(position))
            f.write('new position:{0},{1}\n'.format(x_new, y_new))
            return "detected"

#發現目標物後回傳無人機目前位置，計算其他無人機需要到的位置及位姿調整，再進行目標定位偵測
def calculateUavPose(targetPos,dronenumber,distance):#目前預設無人機數量不超過3部
    #distance-=3        #高度30公尺就設距離相距30#高度20就設定距離25或20#高度10就設相距15
    multiPosList=[]
    #slobedistance1 = ((distance ** 2) / 2) ** 0.5
    #slobedistance2= ((slobedistance1 ** 2)/2) ** 0.5
    if dronenumber==1:
        xpos=targetPos[0]+distance
        ypos=targetPos[1]+distance
        zpos=targetPos[2]
        multiPosList.append(airsim.Vector3r(xpos,ypos,zpos))
    elif dronenumber==2:
        xpos=targetPos[0]-distance
        ypos=targetPos[1]-distance
        zpos=targetPos[2]
        multiPosList.append(airsim.Vector3r(xpos,ypos,zpos))
    elif dronenumber==3:
        xpos = targetPos[0]-distance
        ypos = targetPos[1]-distance#80 -30m#113-40m
        zpos = targetPos[2]
        multiPosList.append(airsim.Vector3r(xpos, ypos, zpos))
        xpos = targetPos[0]+distance  #60-30m
        ypos = targetPos[1]-distance  #+
        zpos = targetPos[2]
        multiPosList.append(airsim.Vector3r(xpos, ypos, zpos))
    return multiPosList

def Go2target(uavpos,posList,detected_dronename,uavList,fov,targetPos,distance,detectList,ws):
    otherUavList=uavList
    for uavname in uavList:
        if uavname==detected_dronename:
            otherUavList.remove(uavname)
            xpos=targetPos[0]+distance
            ypos=targetPos[1]+distance
            client.moveToPositionAsync(xpos, ypos, targetPos[2], 2, vehicle_name=uavname).join()
            print("{0} reach to the target nearby point".format(uavname))
            for k in range(len(otherUavList)):
                assitUavName=otherUavList[k]
                client.moveToPositionAsync(posList[k].x_val, posList[k].y_val, posList[k].z_val, 2,vehicle_name=assitUavName).join()
                print("{0} reach to the target nearby point".format(assitUavName))
    print('all uav is reach to the target nearby position')
    time.sleep(2)
    Uav_list = client.listVehicles()
    for name in Uav_list:
        yaw=0
        for k in range(19):
            yaw-=20#30
            client.rotateToYawAsync(yaw=yaw, timeout_sec=5, margin=5, vehicle_name=name)
            time.sleep(1)
            detectCount = 0
            while detectCount < 3:
                detectCount += 1
                for l in range(15):
                    response = targetDetect_search(cameraName="0", ImageType=airsim.ImageType.Scene, drone=name,
                                                   Fov=fov, Folderpath=savepath, FrameWidth=FrameWidth,
                                                   detectList=detectList, ws=ws)
                    if response == "detected":
                        break
            print("{0}-{1}time:turn -35 degrees".format(name,k+1))
        print("{0} end detect".format(name))
        cv2.destroyAllWindows()

# worker process initialization
def init_worker(arg_queue):
    # define global variable for each worker
    global queue
    # store queue in global argument
    queue = arg_queue

cx = 0
cy = 0
SearchRadius = 150 # 搜尋範圍估算的方形半徑
searchStripwidth = 10  # 平行搜尋的寬度範圍
CAMERA_NAME = 'front_center'  # 相機名稱(拍攝視角)
IMAGE_TYPE = airsim.ImageType.Scene  # 影像類型
FieldOfView =60  # 拍攝視角
FrameWidth = 1024 # 影像畫素(可提升畫素，高度較高也可辨識目標物)
uav_Altitude = 10# 飛行高度
uav_Velocity = 2# 飛行速度
Folder=time.time()
distanceOverTarget=40#fov=60 height=20
if FieldOfView==60:
    distanceOverTarget=55#65 #test height=30 fov=60 高度越高距離要越遠##10m-40m20m-60m
elif FieldOfView==90:
    distanceOverTarget=45 #test height=30 fov=60 高度越高距離要越遠
elif FieldOfView==120:
    distanceOverTarget=40#just is20 need to know far or near

columns = ["timestamp","UavAmount","uavName", "searchTimeSpan", "Xt", "Yt", "realXt", "realYt", "height",
           "velocity", "fov", "framewidth" ,"uav_x", "uav_y", "uav_z", "depth", "slobe", "distanceOverUav"]
# Execute main
detectAllList=columns
r_detect=[]

if __name__ == '__main__':
    client=airsim.MultirotorClient()
    Uav_list = client.listVehicles()
    M=(len(Uav_list))
    Uav_flyHeight = "Uavs{0}-h{1}v{2}Fov{3}FW{4}-{5}".format(M,uav_Altitude, uav_Velocity, FieldOfView, FrameWidth ,Folder)  # 數據資料儲存夾路徑命名
    savepath = r'.\datasetGPS\{0}'.format(Uav_flyHeight)
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.append(columns)
    detectAllList[1] = M
    detectAllList[17] = distanceOverTarget
    detectAllList[8] = uav_Altitude
    detectAllList[9] = uav_Velocity
    detectAllList[10] = FieldOfView
    detectAllList[11] = FrameWidth
    if not os.path.exists(savepath):
        os.mkdir(savepath)
    detetResult=[]
    TimeRecordpath = r"{0}\timeSpan.txt".format(savepath)
    targetRealpos = client.simGetObjectPose(object_name='person4').position
    with open(TimeRecordpath, 'a') as f:
        f.write("{0}\r\n".format(targetRealpos))
    print(targetRealpos)
    detectAllList[6]=targetRealpos.x_val
    detectAllList[7]=targetRealpos.y_val
    routelist=[]
    routePlan=[]
    surveytime = 0
    if M==1:
        routePlan=dividuleTrajectory(cx,cy,uav_Altitude, uav_Velocity, searchStripwidth, SearchRadius)
        routelist.append(routePlan[0])
        surveytime = routePlan[1]
    elif M%2==0:
        routePlan= evenTrajectory(cx,cy,uav_Altitude, uav_Velocity, searchStripwidth, SearchRadius, M)
        for i in range(M):
            routelist.append(routePlan[0][i])
        surveytime = routePlan[1]
    elif M%2==1 and M>=2:
        routePlan=oddTrajectory(cx,cy,uav_Altitude, uav_Velocity, searchStripwidth, SearchRadius, M)
        for i in range(M):
            routelist.append(routePlan[0][i])
        surveytime = routePlan[1]
    print(routelist)
    routePool=Pool(M)
    for o in range(M):
        name=Uav_list[o]
        flytoStartpoint(routelist[o], uav_Velocity, name,)
    routePool.close()
    routePool.terminate()
    queue = Queue()
    pool=Pool(initializer=init_worker,initargs=(queue,),processes=M*3)
    startTime=time.time()
    for m in range(6):
        detetResult.append(False)
    for i in range(len(Uav_list)):
        name = Uav_list[i]
        pool.apply_async(startSurey, args=(routelist[i], uav_Velocity, name, surveytime,))
        #pool.apply_async(getPostion, args=(name,))
        items=pool.apply_async(targetDetect,(CAMERA_NAME, IMAGE_TYPE,name, FieldOfView, savepath, FrameWidth, detectAllList,))
    detetResult=queue.get()
    print('done detected')
    pool.terminate()
    endTime = time.time()
    totalTime = endTime - startTime
    print('total survey time is {0} sec'.format(endTime - startTime))
    detectAllList[3] = totalTime
    detetResult[5][3] = totalTime
    detetResult[3]=((detetResult[3]**2)/2)**0.5  #other height
    #detetResult[3] = ((96 ** 2) / 2) ** 0.5#40meter
    ws.append(detetResult[5])
    with open(TimeRecordpath, 'a') as f:
        f.write('\n\rtotal detect time from startpoint is:{0}sec\n'.format(totalTime))
    targetEstimatePos = detetResult[4]
    MYLIST = calculateUavPose(targetEstimatePos, M,
                              detetResult[3])  # arg1:目前預測的目標物位置;arg2:無人機數量;arg3:與目標物距離
    Go2target(detetResult[2], MYLIST, detetResult[1], Uav_list, fov=FieldOfView,
              targetPos=targetEstimatePos,
              distance=detetResult[3], detectList=detectAllList, ws=ws)
    wb.save(r'.\datasetGPS\{0}.xlsx'.format(Uav_flyHeight))
    wb.close()