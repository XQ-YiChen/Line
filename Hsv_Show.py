import cv2 as cv 
import numpy as np

#定义空函数，防止函数异常
def nothing (*arg):
    pass

def Mask_White(frame,Lower_White,High_White):
    #高斯滤波去除部分噪声
    frame_BGR = cv.GaussianBlur(frame,(7,7),0)
    #将BGR图像转换成HSV图像
    HSV = cv.cvtColor(frame_BGR,cv.COLOR_BGR2HSV)
    #创建对白色颜色范围的掩模
    White_Mask =cv.inRange(HSV,Lower_White,High_White)
    #形态学处理，再次去除噪声
    kernal = cv.getStructuringElement(cv.MORPH_ELLIPSE,(7,7))
    #闭运算，将确实孔洞补充
    White_Mask = cv.morphologyEx(White_Mask,cv.MORPH_CLOSE,kernal)
    #开训算，掩膜的边缘更加平滑
    White_Mask = cv.morphologyEx(White_Mask, cv.MORPH_OPEN,kernal)
    return White_Mask
    
#初始化摄像头
cap =cv.VideoCapture(0)
#创建滑动条窗口
cv.namedWindow('White Range',cv.WINDOW_AUTOSIZE)
cv.createTrackbar('Hue Low','White Range',0,180,nothing)
cv.createTrackbar('Hue High','White Range',180,180,nothing)
cv.createTrackbar('Sat Low','White Range',0,255,nothing)
cv.createTrackbar('Sat High','White Range',30,255,nothing)
cv.createTrackbar('Val Low','White Range',200,255,nothing)
cv.createTrackbar('Val High','White Range',255,255,nothing)

while True:
    ret,frame =cap.read()
    if not ret:
        print('can not recive frame,Exiting...')
        break

    #色调
    Hue_Low = cv.getTrackbarPos('Hue Low','White Range')
    Hue_High = cv.getTrackbarPos('Hue High','White Range')
    #饱和度
    Sat_Low = cv .getTrackbarPos('Sat Low','White Range')
    Sat_High =cv.getTrackbarPos('Sat High','White Range')
    #亮度
    Val_Low =cv.getTrackbarPos('Val Low','White Range')
    Val_High =cv.getTrackbarPos('Val High','White Range')
    
    Lower_White =np.array([Hue_Low,Sat_Low,Val_Low])
    Upper_White =np.array([Hue_High,Sat_High,Val_High])
    
    img = Mask_White(frame,Lower_White,Upper_White)
    cv.imshow('Original',frame)
    cv.imshow('Mask Img',img)
    
    if cv.waitKey(1) &0xFF ==ord('q'):
        break
    
cap.release()
cv.destroyAllWindows()