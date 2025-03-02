# -*- coding: utf-8 -*-
import cv2 as cv 
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

#定义空函数，防止函数异常
def nothing (*arg):
    pass

class Ucar_opencv:
    def __init__(self):
        rospy.init_node('ucar_opencv',anonymous=True)
        self.pubilsher_cmd = rospy.Publisher('/car_cmd_vel',Twist,queue_size=10)
        #设置发布频率
        self.rate = rospy.Rate(10)#10Hz
        #初始化Twist消息
        self.cmd_vel=Twist()
        self.cmd_vel.linear.x = 1.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        #定义白色车道线HSV范围
        self.Lower_White =np.array([0,0,200])
        self.Upper_White =np.array([180,30,255])
        #初始化扫线起点
        self.Init = 320
        #初始化摄像头
        self.cap = cv.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr('can not recive frame,Exiting...')
            exit(1)
    
    
    def Mask_White(self,frame):
        #高斯滤波去除部分噪声
        frame_BGR = cv.GaussianBlur(frame,(7,7),0)
        #将BGR图像转换成HSV图像
        HSV = cv.cvtColor(frame_BGR,cv.COLOR_BGR2HSV)
        #创建对白色颜色范围的掩模
        White_Mask =cv.inRange(HSV,self.Lower_White,self.Upper_White)
        #形态学处理，再次去除噪声
        kernal = cv.getStructuringElement(cv.MORPH_ELLIPSE,(7,7))
        #闭运算，将确实孔洞补充
        White_Mask = cv.morphologyEx(White_Mask,cv.MORPH_CLOSE,kernal)
        #开训算，掩膜的边缘更加平滑
        White_Mask = cv.morphologyEx(White_Mask, cv.MORPH_OPEN,kernal)
        return White_Mask
    
    
    def Mid_Line(self,img):
        #记录左侧车道线的像素点
        Left_Line = np.array([])
        #记录右侧车道线的像素点
        Right_Line = np.array([])
        #备份初始图像的车道线像素点
        Left_Line_B = np.array([])
        Right_Line_B = np.array([])
        #记录拟合中线的车道线像素点
        Medim_Line = np.array([])
        #记录初始扫线的坐标
        Midline_y =self.Init
        Res = np.array([])
    
        #开始扫线
        for i in range(479,0,-1):
            #开始扫左线
            flag = False
            for j in range (Midline_y+1,1,-1):
                if(img[i][j]==0)and(img[i][j-1]==255):
                    #记录左线跳变点的值
                    flag =True
                    Left_Line = np.append(Left_Line,j)
                    Left_Line_B = np.append(Left_Line_B,j)
                    Left = j
                    break
        
            if flag == False:
                Left_Line = np.append(Left_Line,0)
                Left_Line_B = np.append(Left_Line_B,0)
            
            #开始扫右线
            flag =False
            for j1 in range (Midline_y+2,638,1):
                if(img[i][j1]==0)and(img[i][j1+1]):
                    #记录右线跳变点的值
                    flag = True
                    Right_Line = np.append(Right_Line,j1)
                    Right_Line_B = np.append(Right_Line_B,j1)
                    Right =j1
                    break    
        
            if flag == False:
                Right_Line = np.append(Right_Line,639)
                Right_Line_B = np.append(Right_Line_B,639)
        
            #若为左拐弧线
            if(Left_Line[479-i]== 0 and Right_Line[479-i]!=639 and img [i-2][0]!= 0):#可能出现越界问题
                break
            #若为右拐弧线
            if(Right_Line[479-i] == 639 and Left_Line[479-i] !=0 and img [i-2][639]!=255):
                break
        
            Midline_y = int((Left_Line[479-i]+Right_Line[479-i])/2)
            Res = np.append(Res,Midline_y)
    
        #开始计算中线坐标
        Medim_Line=(Left_Line_B+Right_Line_B)/2
        #检查数组长度是否一致
        if(len(Left_Line_B)!=len(Right_Line_B)):
            return ValueError("左右车道线坐标数组的长度不一致")
        #开始遍历绘制
        for k in range(len(Left_Line_B)-1,-1,-1):
            #计算中线点坐标
            Point_Mid = (int(Medim_Line[k]),479-k)
            #在图上绘制点
            cv.circle(img,Point_Mid,1,(255,0,255),-1)
    
        cv.imshow('Res',img)
        cv.waitKey(1)
    
        return img,Medim_Line
    
    #计算中线与图像中心线的偏差
    def calculate_deviation(self,medim_line,img_width):
        center_line = img_width//2
        deviation = np.mean(medim_line)-center_line
        return deviation
    
    #根据角度偏差调整车辆方向
    def control_car(self,deviation):
        if deviation < -5:
            self.cmd_vel.angular.z = -0.2 #实现右转
        elif deviation > 5 :
            self.cmd_vel.angular.z = 0.2 #实现左转
        else :
            self.cmd_vel.angular.z = 0.0 #直行  
            
    #主循环
    def run (self):
        while not rospy.is_shutdown():
            ret , frame =self.cap.read()
            if not ret:
                rospy.logerr('can not recive frame,Exiting...')
                break
            
            #处理摄像头捕捉到的图像
            img = self.Mask_White(frame)
            img,medim_line = self.Mid_Line(img)
            deviation = self.calculate_deviation(medim_line,img.shape[1])
            self.control_car(deviation)
            
            #发布速度指令
            self.pubilsher_cmd.publish(self.cmd_vel)
            rospy.loginfo(f"发布速度命令:linear.x={self.cmd_vel.linear.x},angular.z={self.cmd_vel.angular.z}")
            
            #输出图像信息
            cv.imshow('Original',frame)
            cv.imshow('Img',img)
            if cv.waitKey(1)&0xFF == ord('q'):
                break
            
            #休眠
            self.rate.sleep()
        
        #释放资源
        self.cap.release()
        cv.destroyAllWindows()

#设置标准入口点
if __name__ == '__main__':
    try:
        node = Ucar_opencv()
        node.run()
    #异常处理
    except rospy.ROSInterruptException:
        pass
