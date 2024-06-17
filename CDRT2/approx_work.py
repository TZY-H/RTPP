import time
import cv2
import socket
import json

#python3.7 opencv3.4.9
# def get_approx_list(approxs, hchy):
#     returnlist = []
#     for i in range(0, len(hchy)):
#         hchylayer = 0
#         hchyp = i
#         while hchy[hchyp][3] != -1:
#             hchyp = hchy[hchyp][3]
#             hchylayer += 1
#         if (hchylayer % 2 == 0):
#             temp_list = approxs[i].tolist()
#             hchyp = hchy[i][2]
#             if hchyp != -1:
#                 temp_list.append([-1, -1])
#                 temp_list += approxs[hchyp].tolist()
#                 while hchy[hchyp][0] != -1:
#                     hchyp = hchy[hchyp][0]
#                     temp_list.append([-1, -1])
#                     temp_list += approxs[hchyp].tolist()
#             returnlist.append(temp_list)
#     return returnlist  # 返回拆分后连通性列表
def get_approx_list(approxs, hchy):
    returnlist = []
    for i in range(0, len(hchy)):
        hchylayer = 0
        hchyp = i
        while hchy[hchyp][3] != -1:
            hchyp = hchy[hchyp][3]
            hchylayer += 1
        if (hchylayer % 2 == 0):
            temp_list = [approxs[i].tolist()]
            hchyp = hchy[i][2]
            if hchyp != -1:
                # temp_list.append([-1, -1])
                # if len(approxs[hchyp]) > 2:
                temp_list.append(approxs[hchyp].tolist())
                while hchy[hchyp][0] != -1:
                    hchyp = hchy[hchyp][0]
                    # if len(approxs[hchyp]) > 2:
                    temp_list.append(approxs[hchyp].tolist())
            returnlist.append(temp_list)
    return returnlist  # 返回拆分后连通性列表


def get_approx(img, robotpixel):
    binary, contour, hierarchy = cv2.findContours(
        img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(len(contour))
    # print(hierarchy)
    approxs = [[]] * len(contour)
    hchy = [[]] * len(contour)
    epsilon = int(robotpixel / 3)
    # print(hierarchy)
    for i in range(0, len(contour)):
        approx = cv2.approxPolyDP(contour[i], epsilon, True)
        approxs[i] = approx[:, 0, :][::-1]
        hchy[i] = hierarchy[0][i]
    return get_approx_list(approxs, hchy)


def approx_work(IMGmap, ratio, robotsize, thresh):
    img_gray = cv2.cvtColor(IMGmap, cv2.COLOR_BGR2GRAY)
    ret, img_bin = cv2.threshold(img_gray, thresh, 255, cv2.THRESH_BINARY)
    robotpixel = int(robotsize / ratio)
    erode_r = int(robotpixel / 2)
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_r, erode_r))
    erosion = cv2.erode(img_bin, element, iterations=1)  # 腐蚀
    return [get_approx(erosion, robotpixel), list(img_bin.shape)]


if __name__ == "__main__":
    approx_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    approx_udp.bind(("0.0.0.0", 23231))
    while True:
        print("wite data!")
        [data, appaddr] = approx_udp.recvfrom(102400)
        try:
            t0 = time.time()
            print(data)
            data = json.loads(data)
            path = data["path"]
            ratio = data["ratio"]
            robotsize = data["robotsize"]
            thresh = data["thresh"]

            IMGmap = cv2.imread(path)
            # cepointlist, mapshape = approx_work(IMGmap, 0.01, 0.3, 127)
            returndata = approx_work(IMGmap, ratio, robotsize, thresh)

            cepointlists = json.dumps(returndata)
            # 将 ceopointlists 写入文件
            # with open('cepointlists.json', 'w') as file:
            #     file.write(cepointlists)

            print("Processing time: ",time.time()-t0,"s")
            approx_udp.sendto(cepointlists.encode("utf-8"),appaddr)
            print("send data len: ",len(cepointlists))
            print("map shape: ",returndata[1])
        except:
            print("receive error!")
