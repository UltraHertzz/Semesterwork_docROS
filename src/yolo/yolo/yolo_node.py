# if Warning is shownd, use command to set environment variable: export PATH="/home/orangepi/.local/bin:$PATH"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from typing import Tuple, Dict
import cv2
import numpy as np
import torch
from ultralytics.utils.plotting import colors



class YoloNode(Node):
    
    def __init__(self,model_name="yolov8n"):
        super().__init__('yolo_node')
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw',self.img_process_callback,10)
        self.publisher = self.create_publisher(Image,'/obj_detect',10)
        self.bridge = CvBridge()
        self.model = YOLO(f'{model_name}.pt')
        
    def plot_one_box(self, box:np.ndarray, img:np.ndarray, mask:np.ndarray = None, label:str = None, line_thickness:int = 5):
        """
        Helper function for drawing single bounding box on image
        Parameters:
            x (np.ndarray): bounding box coordinates in format [x1, y1, x2, y2]
            img (no.ndarray): input image
            color (Tuple[int, int, int], *optional*, None): color in BGR format for drawing box, if not specified will be selected randomly
            mask (np.ndarray, *optional*, None): instance segmentation mask polygon in format [N, 2], where N - number of points in contour, if not provided, only box will be drawn
            label (str, *optonal*, None): box label string, if not provided will not be provided as drowing result
            line_thickness (int, *optional*, 5): thickness for box drawing lines
        """
        # Plots one bounding box on image img
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = [np.random.randint(0, 255) for _ in range(3)]
        # color = colors[label]
        c1, c2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
        if mask is not None:
            image_with_mask = img.copy()
            mask
            cv2.fillPoly(image_with_mask, pts=[mask.astype(int)], color=color)
            img = cv2.addWeighted(img, 0.5, image_with_mask, 0.5, 1)
        return img


    def draw_results(self, results, source_image:np.ndarray, label_map:Dict):
        """
        Helper function for drawing bounding boxes on image
        Parameters:
            image_res (np.ndarray): detection predictions in format [x1, y1, x2, y2, score, label_id]
            source_image (np.ndarray): input image for drawing
            label_map; (Dict[int, str]): label_id to class name mapping
        Returns:
            image with bounding box and confidence (type: np.ndarray)

        """
        #masks = results.get("segment")
        h, w = source_image.shape[:2]
        for i in range(results.size(0)):
            label = f'{label_map[int(results[i,5])]} {results[i,4]:.2f}'
            #mask = masks[idx] if masks is not None else None
            source_image = self.plot_one_box(results[i,:4], source_image, label=label, line_thickness=1)
        return source_image

    def img_process_callback(self, data):
        """
        process the image from realsense camera, draw the anchor box and publish to topic "obj_detect" (type: Sensor_msgs.msg.Image)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.label_map = self.model.model.names
        res = self.model(self.cv_image)
        
        ###
        annotated_res = res[0].plot()
        img_pub = self.bridge.cv2_to_imgmsg(annotated_res,'bgr8')
        self.publisher.publish(img_pub)
        ###
            
        # bounding_box = res[0].boxes.xyxy # Shape = (num_obj,4) 
        # box_num = bounding_box.shape[0]  # num_obj
        # if box_num != 0:
            # score = np.reshape(res[0].boxes.conf,(box_num,-1))
            # box_class = np.reshape(res[0].boxes.cls,(box_num,-1))
            # self.obj_list = torch.concat((bounding_box,score,box_class),dim=1)
            # result_img = self.draw_results(self.obj_list,self.cv_image,self.label_map)
            # 
            # img_pub = self.bridge.cv2_to_imgmsg(result_img,'bgr8')
            # self.publisher.publish(img_pub)
        # else:
            # print("no object found!")

def main(args=None):
    rclpy.init(args=args)
    yoloNode = YoloNode()
    rclpy.spin(yoloNode)
    yoloNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
