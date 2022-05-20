'''
Description: 
Version: 2.0
Date: 2022-05-11 11:46:01
LastEditors: Meroke
LastEditTime: 2022-05-18 20:52:07
Author: Meroke 3154911544@qq.com
FilePath: /python/graph_enhance.py

Copyright (c) 2022 by Meroke 3154911544@qq.com, All Rights Reserved. 
'''

import math
import re
import sys
from this import d
from turtle import resizemode

import cv2 as cv
import numpy as np

image_height = 128
image_width = 64


'''
@description:  将图像等比例放大显示在窗口中
param {*} img 传入opencv读取的图片
param {*} name 窗口名
param {*} times 放大倍数
return {*}
'''
def resize_imshow(img,name,times=5):
    global image_height, image_width
    height = image_height * times
    width  =image_width *times
    dst = cv.resize(img,(height,width))
    # dst = img
    cv.imshow(name,dst)



'''
@description: 水平显示两张图片
param {*} img1 图片1
param {*} img2 图片2
return {*}
'''
def hstack_show(img1,img2):
    img_hor = np.hstack((img1,img2))
    img_hor = cv.resize(img_hor,(1024*2,512))
    cv.imshow("hstack",img_hor)
'''
@description: 垂直显示两张图片
param {*} img1 图片1
param {*} img2 图片2
return {*}
'''
def vstack_show(img1,img2):
    img_ver = np.vstack((img1,img2))
    img_ver = cv.resize(img_ver,(1024,512*2))
    cv.imshow("hstack",img_ver)


'''
@description: 连接所有轮廓点对应的线段
param {*} img  原图
param {*} list 包含四个列表的大列表，每个小列表包含一边的轮廓点
param {*} color 线段颜色，默认蓝色
return {*}
'''
def draw_lines(img,list,color=(255,0,0) ):
    for j in range(len(list)):
        for i in range(len(list[j])-1):
            x,y = list[j][i][0],list[j][i][1]
            x2,y2 = list[j][i+1][0],list[j][i+1][1]
            cv.line(img,(int(x),int(y)),(int(x2),int(y2)),color,1) # 蓝

'''
@description: 将原图地图清楚，形成空白模板，用于重新绘制新地图
param {*} img 原图
return {*}
'''
def return_blank(img):
    print(img)
    for y in range(img.shape[1]):
        for x in range(img.shape[0]):
            if(img[x,y]==0 ):
                img[x,y] = 255
    return img


# 计算轮廓面积，被调用排序轮廓
def cnt_area(cnt):
  area = cv.contourArea(cnt)
  return area

# 列表排序函数，选择元素作为排序依据
def sort_fir(list):
    return list[0]
def sort_sec(list):
    return list[1]

'''
@description: 划分轮廓的四边的点，并自动排序
param {*} points 轮廓点列表
param {*} mius 位移值，原轮廓与新轮廓的距离差值
return {*}
'''
def get_points_list(points,mius=1):
    l1 = [] # 左
    l2 = [] # 右
    l3 = [] # 上
    l4 = [] # 下

    for point in points:
        if(point[0][0]<18):
            l1.append( (point[0][0]-mius,point[0][1]) )
        elif(point[0][0]>110):
            l2.append( (point[0][0]+mius,point[0][1]) )
        elif(point[0][1]<16):
            l3.append( (point[0][0],point[0][1]-mius) )
        else:
            l4.append( (point[0][0],point[0][1]+mius) )
    
    l1.sort(key=sort_sec)
    l2.sort(key=sort_sec)
    l3.sort(key=sort_fir)
    l4.sort(key=sort_fir)

    return [l1,l2,l3,l4]

'''
@description: 地图优化器-针对盒子分离效果强化
param {*} img  原图
return {*}
'''
def split_boxs(img):
    # 中间
    img[45:53,63:65] = 255
    # 右边
    img[45:53,96:99] = 255
    # 左侧
    img[45:53,30:33] = 255

    return img

def save_result(img):
    h,w = img.shape[0:1]
    resulotion = 0.005
    


'''
@description: 总流程处理函数    
    目前方法: 轮廓重绘地图
    1.通过erode后的pgm地图,扫描多层轮廓，获取最内层轮廓
    2.由于是最内层的轮廓与原图存在一点距离，于是再分四个方向划分轮廓
    3.将四个方向的轮廓像素点 沿对应方向位移即可获得最简洁有效的地图
param {*} img  原图
return {*}
'''
def del_7(img):
    kernel_big = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    kernel_small = cv.getStructuringElement(cv.MORPH_RECT, (4,4))
    img = cv.erode(img,kernel_big)
    resize_imshow(img,"erode")
    img = split_boxs(img)
    cv.imshow("img_test",img)
    img = cv.dilate(img,kernel_small)
    resize_imshow(img,"dilate")

    ret,gray  = cv.threshold(img,170,255,cv.THRESH_BINARY_INV)
    resize_imshow(gray,"gray")

    contours, hierarchy = cv.findContours(gray, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
    contours.sort(key=cnt_area, reverse=True)  # 自大到小排序
    blank = return_blank(img)
    blank = cv.cvtColor(blank,cv.COLOR_GRAY2BGR)
    if(len(contours) > 1 and len(contours) <3):
        # cv.drawContours(blank, [contours[1]], -1, (0, 0, 255), 1 )  # 红
        cv.drawContours(blank, [contours[1]], -1, (0, 0, 0), 1 )  # 黑
        list = get_points_list(contours[1],1)
        
        # draw_lines(blank,list,(255,0,0)) # 蓝
        draw_lines(blank,list,(0,0,0)) # 黑

        # 位移后效果对比,测试显示红蓝线段，最终显示黑色
        resize_imshow(blank,"contonurs")
        
    elif contours:
        cv.drawContours(blank, contours, -1, (0, 255, 0), 1 ) # 绿
        resize_imshow(blank,"contonurs")
    # del_3(blank)
    cv.imshow("result",blank)
    
if __name__  == "__main__":
    img = cv.imread("/home/eaibot/2022_jsjds/dashgo_ws/src/dashgo/dashgo_nav/2022_map/test.pgm",0)
    resize_imshow(img,'test.pgm')
    del_7(img)
    while(1):
        k = cv.waitKey(0)
        if(k == ord('q')):
            break
    cv.destroyAllWindows()
