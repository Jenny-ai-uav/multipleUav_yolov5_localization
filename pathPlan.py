import math,airsim
#M=4
import time,threading

def evenTrajectory(cx,cy,altitude,velocity,stripwidth,Radius,M):
    trajectLogL = []
    trajectLogR=[]
    trajectoryLog=[]
    print("start calculate the survey path")
    # AirSim uses NED coordinates so negative axis is up.
    z = -altitude
    number = int(M / 2)
    maxLOOP = Radius / (2 * stripwidth)
    for i in range(1,3):
        trajectLog = []
        path = []
        distance = 0
        if i % 2 == 0:  # 若為雙數，向右飛行
            iteration = 1
            startX = cx - Radius
            startY = cy + stripwidth
            # 基準點由中心先向左右各stripwidth後開始描繪
            # startposition = airsim.Vector3r(startX, startY, z)
            # path.append(startposition)
            trajectLog.append([startX, startY, z])
            path.append(airsim.Vector3r(startX, startY, z))
            #path.append(airsim.Vector3r(startX, startY, z))
            while iteration < maxLOOP:
                iteration += 1
                startX += (Radius * 2)
                startY = startY
                distance += (Radius * 2)
                # path.append(airsim.Vector3r(startX, startY, z))
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
                startX = startX
                startY += stripwidth
                distance += stripwidth
                # path.append(airsim.Vector3r(startX, startY, z))
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
                startX -= (Radius * 2)
                startY = startY
                distance += (Radius * 2)
                # path.append(airsim.Vector3r(startX, startY, z))
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
                startX = startX
                startY += stripwidth
                distance += stripwidth
                # path.append(airsim.Vector3r(startX, startY, z))
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
            splitNum = int(len(trajectLog)/number)
            kR = -1
            for j in range(int(M / 2)):
                trajectLogR.append(path[kR + 1:splitNum + kR + 1])
                kR += splitNum
                print("routeR{0}:{1}".format(j, trajectLogR[j]))
                trajectoryLog.append(trajectLogR[j])
            if M>2:
                distance = distance*2 /M
            else:
                distance=distance
            trip_time = distance / velocity
        else:  # 若為單數，向左飛行
            iteration = 1
            startX = cx - Radius
            startY = cy - stripwidth
            trajectLog.append([startX, startY, z])
            path.append(airsim.Vector3r(startX, startY, z))
            while iteration < maxLOOP:
                iteration += 1
                startX += (Radius * 2)
                startY = startY
                distance += (Radius * 2)
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
                startX = startX
                startY -= stripwidth
                distance += stripwidth
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
                startX -= (Radius * 2)
                startY = startY
                distance += (Radius * 2)
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
                startX = startX
                startY -= stripwidth
                distance += stripwidth
                trajectLog.append([startX, startY, z])
                path.append(airsim.Vector3r(startX, startY, z))
            splitNum = int(len(path) /number)
            k = -1
            for j in range(int(M / 2)):
                trajectLogL.append(path[k + 1:splitNum + k + 1])
                k += splitNum
                print("routeL{0}:{1}\n\r".format(j, trajectLogL[j]))
                trajectoryLog.append(trajectLogL[j])
            if M>2:
                distance = distance*2 /M
            else:
                distance=distance
            trip_time = distance / velocity
    return [trajectoryLog, trip_time]

def oddTrajectory(cx,cy,altitude,velocity,dw,dh,M):
    z=-altitude
    trajectLog = []
    pathCenter=[]
    pathCenter_3d=[]
    pathL=[]
    pathL_3d = []
    pathR=[]
    pathR_3d=[]
    pathL_split=[]
    pathR_split=[]
    loopC=(dh*2/(dw*M*2))
    loopOther=((dh*2-2*dw*loopC)/(2*2*dw))
    number = int((M-1) / 2)
    distance = 0
    for i in range(3):
        iteration = 1
        if i==0:
            startX=cx-dh
            startY=int(cy-(dh/M))####要解決起始點相同會碰撞的問題
            #pathCenter.append([startX, startY, z])
            #pathCenter_3d.append(airsim.Vector3r(startX, startY, z))
            while iteration<=loopC:
                iteration+=1
                startX=startX
                startY+=dw
                distance+=dw
                pathCenter.append([startX, startY, z])
                pathCenter_3d.append(airsim.Vector3r(startX, startY, z))
                startX+=dh*2
                startY=startY
                distance += 2*dh
                pathCenter.append([startX, startY, z])
                pathCenter_3d.append(airsim.Vector3r(startX, startY, z))
                startX=startX
                startY+=dw
                distance += dw
                pathCenter.append([startX, startY, z])
                pathCenter_3d.append(airsim.Vector3r(startX, startY, z))
                startX-=2*dh
                startY=startY
                distance += 2*dh
                pathCenter.append([startX, startY, z])
                pathCenter_3d.append(airsim.Vector3r(startX, startY, z))
            print("centerRoute:\r\n{0}\n\r".format(pathCenter))
            trajectLog.append(pathCenter_3d)
            triptime=distance/velocity
        if i==1:#left
            startX=cx-dh
            startY=int(cy-(dh/M))
            pathL.append([startX, startY, z])
            pathL_3d.append(airsim.Vector3r(startX, startY, z))
            while iteration<=loopOther:
                iteration+=1
                startX+=(dh*2)
                startY=startY
                #distance+=2*dh
                pathL.append([startX, startY, z])
                pathL_3d.append(airsim.Vector3r(startX, startY, z))
                startX=startX
                startY-=dw
                #distance += dw
                pathL.append([startX, startY, z])
                pathL_3d.append(airsim.Vector3r(startX, startY, z))
                startX-=(2*dh)
                startY=startY
                #distance += (2*dh)
                pathL.append([startX, startY, z])
                pathL_3d.append(airsim.Vector3r(startX, startY, z))
                startX=startX
                startY-=dw
                #distance += dw
                pathL.append([startX, startY, z])
                pathL_3d.append(airsim.Vector3r(startX, startY, z))
            splitNum = math.ceil(len(pathL_3d) / number)
            if M>3:
                k=-1
                for j in range(int((M-1) / 2)):
                    pathL_split.append(pathL_3d[k + 1:splitNum + k + 1])
                    k += splitNum
                    print("routeL-{0}:\r\n{1}\n\r".format(j, pathL_split[j]))
                    trajectLog.append(pathL_split[j])
            else:
                print("leftRoute:\r\n{0}\n\r".format(pathL_3d))
                trajectLog.append(pathL_3d)
        if i==2:#right
            startX=cx-dh
            startY=int(cy+(dh/M))
            pathR.append([startX, startY, z])
            pathR_3d.append(airsim.Vector3r(startX, startY, z))
            while iteration<=loopOther:
                iteration+=1
                startX+=(dh*2)
                startY=startY
                #distance+=2*dh
                pathR.append([startX, startY, z])
                pathR_3d.append(airsim.Vector3r(startX, startY, z))
                startX=startX
                startY+=dw
                #distance += dw
                pathR.append([startX, startY, z])
                pathR_3d.append(airsim.Vector3r(startX, startY, z))
                startX-=(2*dh)
                startY=startY
                #distance += (2*dh)
                pathR.append([startX, startY, z])
                pathR_3d.append(airsim.Vector3r(startX, startY, z))
                startX=startX
                startY+=dw
                #distance += dw
                pathR.append([startX, startY, z])
                pathR_3d.append(airsim.Vector3r(startX, startY, z))
            splitNum = math.ceil(len(pathR_3d) / number)
            if M>3:
                k=-1
                for j in range(int((M-1) / 2)):
                    pathR_split.append(pathR_3d[k + 1:splitNum + k + 1])
                    k += splitNum
                    trajectLog.append(pathR_split[j])
                    print("routeR-{0}:\r\n{1}\n\r".format(j, pathR_split[j]))
            else:
                print("rightRoute:\r\n{0}\n\r".format(pathR_3d))
                trajectLog.append(pathR_3d)
    return [trajectLog,triptime]

def dividuleTrajectory(cx,cy,altitude,velocity,dw,dh):
    z = -altitude
    trajectLog = []
    pathLog=[]
    loop = (dh * 2 / (dw  * 2))
    distance = 0
    iteration = 1
    startX = cx - dh
    startY = cy - dh  ####要解決起始點相同會碰撞的問題
    pathLog.append([startX, startY, z])
    trajectLog.append(airsim.Vector3r(startX, startY, z))
    while iteration <= loop:
        iteration += 1
        startX += 2*dh
        startY = startY
        distance += 2*dh
        pathLog.append([startX, startY, z])
        trajectLog.append(airsim.Vector3r(startX, startY, z))
        startX = startX
        startY +=dw
        distance +=dw
        pathLog.append([startX, startY, z])
        trajectLog.append(airsim.Vector3r(startX, startY, z))
        startX -= 2*dh
        startY = startY
        distance += 2*dh
        pathLog.append([startX, startY, z])
        trajectLog.append(airsim.Vector3r(startX, startY, z))
        startX = startX
        startY +=dw
        distance += dw
        pathLog.append([startX, startY, z])
        trajectLog.append(airsim.Vector3r(startX, startY, z))
    print("Route:\r\n{0}\n\r".format(pathLog))
    triptime = distance / velocity
    return [trajectLog, triptime]

cx = 0
cy = 0
dh = 200 # 搜尋範圍估算的方形半徑
dw = 10  # 平行搜尋的寬度範圍
droneNum = 4# 無人機數量
droneNum2 = 3
uav_Altitude = 20  # 飛行高度
uav_Velocity = 1 # 飛行速度

#evenTrajectory(cx,cy,uav_Altitude,uav_Velocity,dw,dh,droneNum)
#print(oddTrajectory(cx,cy,uav_Altitude,uav_Velocity,dw,dh,droneNum2))
#trajec1 = trajectory(cx,cy,uav_Altitude, uav_Velocity, dw, dh, droneNum)
#print(dividuleTrajectory(cx,cy,uav_Altitude,uav_Velocity,dw,dh))
#trajec2 = trajectory(uav_Altitude, uav_Velocity, searchStripwidth, SearchRadius, droneNum2)