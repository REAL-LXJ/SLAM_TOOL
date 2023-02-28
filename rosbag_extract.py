#coding:utf-8
import rosbag
import rospy
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class RosDataCreator():
    def __init__(self):
        self.bridge = CvBridge()
    def _rosbag_extract(self, rosbag_path, 
                            pic0_topic, pic0_path, 
                            pic1_topic, pic1_path, 
                            depth0_topic, depth0_path, 
                            depth1_topic, depth1_path):
                            #imu_topic, imu_path)
                            
        imu_txt = open(imu_path, 'w')

        with rosbag.Bag(rosbag_path, 'r') as bag:   
            for topic,msg,t in bag.read_messages():
                
                #* 左目rgb图像topic
                if topic == pic0_topic:  
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" %  msg.header.stamp.to_sec()           #%.6f表示小数点后带有6位，可根据精确度需要修改
                    image_name = timestr + ".png"                           #% 图像命名：时间戳.png
                    cv2.imwrite(pic0_path + image_name, cv_image)  

                #* 右目rgb图像topic
                if topic == pic1_topic:  
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" %  msg.header.stamp.to_sec()           #%.6f表示小数点后带有6位，可根据精确度需要修改
                    image_name = timestr + ".png"                           #% 图像命名：时间戳.png
                    cv2.imwrite(pic1_path + image_name, cv_image)  
                
                #* 左目深度图像topic 
                if topic == depth0_topic:  
                    try:
                        msg.encoding = "mono16"
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" %  msg.header.stamp.to_sec()           #%.6f表示小数点后带有6位，可根据精确度需要修改
                    image_name = timestr + ".png"                           #% 图像命名：时间戳.png
                    cv2.imwrite(depth0_path + image_name, cv_image)  

                #* 右目深度图像topic
                if topic == depth1_topic:  
                    try:
                        msg.encoding = "mono16"
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" %  msg.header.stamp.to_sec()           #%.6f表示小数点后带有6位，可根据精确度需要修改
                    image_name = timestr + ".png"                           #图像命名：时间戳.png
                    cv2.imwrite(depth1_path + image_name, cv_image)  
                
                #* imu topic
        #         if topic == imu_topic:
        #             ax = "%.6f" % msg.linear_acceleration.x
        #             ay = "%.6f" % msg.linear_acceleration.y
        #             az = "%.6f" % msg.linear_acceleration.z
        #             gx = "%.6f" % msg.angular_velocity.x
        #             gy = "%.6f" % msg.angular_velocity.y
        #             gz = "%.6f" % msg.angular_velocity.z
        #             time = "%.6f" %  msg.header.stamp.to_sec()
        #             imu_data = time + " " + ax + " " + ay + " " + az + " " + gx + " " + gy + " " + gz
        #             imu_txt.write(imu_data)
        #             imu_txt.write('\n')
        # imu_txt.close()
        

if __name__ == '__main__':

    rosbag_path = "/media/lxj/LXJ/oppo/3.bag"                                 #* ros bag包位置

    pic0_path = '/home/lxj/Dataset/pic0/'                                     #* 存放左目rgb图片的位置
    pic1_path = '/home/lxj/Dataset/pic1/'                                     #* 存放右目rgb图片的位置
    depth0_path = '/home/lxj/Dataset/depth0/'                                 #* 存放左目depth图片的位置
    depth1_path = '/home/lxj/Dataset/depth1/'                                 #* 存放右目depth图片的位置
    imu_path = "/home/lxj/Dataset/imu.txt"                                    #* 存放imu的txt文件位置
    pointcloud_path = " "                                                     #* 存放点云的位置



    pic0_topic = "/master/infra1/image_rect_raw"                              #* 左目rgb图片topic 
    pic1_topic = "/slaver/infra1/image_rect_raw"                              #* 右目rgb图片topic                                  
    depth0_topic = "/master/depth/image_rect_raw"                             #* 左目depth图片topic
    depth1_topic = "/slaver/depth/image_rect_raw"                             #* 右目depth图片topic
    imu_topic = "/master/imu"                                                 #* imu topic
    
    try:
        creator = RosDataCreator()
        creator._rosbag_extract( rosbag_path, 
                                pic0_topic, pic0_path, 
                                pic1_topic, pic1_path, 
                                depth0_topic, depth0_path, 
                                depth1_topic, depth1_path)
                                #imu_topic, imu_path)
        print("rosbag data extract complete!")
    except rospy.ROSInterruptException:
        pass
