import airsim,cv2,time
import numpy as np
import detectJ as detect  #可以看是否有精進辨識速度或辨識率的方式
cx = 0
cy = 0
SearchRadius = 100 # 搜尋範圍估算的方形半徑
searchStripwidth = 10  # 平行搜尋的寬度範圍
CAMERA_NAME = 'front_center'  # 相機名稱(拍攝視角)
IMAGE_TYPE = airsim.ImageType.Scene  # 影像類型
fov =60  # 拍攝視角
FrameWidth = 1024 # 影像畫素(可提升畫素，高度較高也可辨識目標物)
uav_Altitude = 40# 飛行高度
uav_Velocity = 2# 飛行速度
Folder=time.time()
distanceOverTarget=40#fov=60 height=20
client = airsim.MultirotorClient()
Uav_list = client.listVehicles()


def parser_detectResult2(detectList):#二次偵測目標物並回傳,閥值0.2
    arr=[]
    for item in detectList:
        if 'person' in item[0] :
            content=item[0].split(' ')
            if float(content[1])>0.2:
                arr.append((item[1],item[2]))
    return arr

def targetDetect_search(cameraName, ImageType, drone,Fov,FrameWidth):
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
        cv2.rectangle(frame, result[0][0], result[0][1], (255, 0, 255), thickness=1, lineType=cv2.LINE_AA)
        cv2.imshow('{0}:scene'.format(drone), frame)  # show original image and detect result
        cv2.waitKey(0)  # 1 millise

for name in Uav_list:
    yaw = 0
    for k in range(10):
        yaw -= 30
        client.rotateToYawAsync(yaw=yaw, timeout_sec=5, margin=5, vehicle_name=name)
        time.sleep(1)
        for l in range(5):
            response = targetDetect_search(cameraName="0", ImageType=airsim.ImageType.Scene, drone=name,
                                           Fov=fov,
                                           FrameWidth=FrameWidth)
            if response == "detected":
                break
        print("{0}-{1}time:turn -30degrees".format(name, k + 1))
    print("{0} end detect".format(name))
    cv2.destroyAllWindows()




