#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bard_msgs.msg import CategorizationBoolArray
#import deeplabv3.msg
import rospkg

import torch
import torch.nn as nn
import argparse
from DeepLabV3Plus import network
from DeepLabV3Plus.network._deeplab import DeepLabHeadV3Plus
from DeepLabV3Plus.datasets import VOCSegmentation, Cityscapes, cityscapes

    
def infer(model, image, mask: list):
    predictions = model(image).max(1)[1].cpu().numpy()[0]
    decode_fn = Cityscapes.decode_target
    colorized_preds = decode_fn(predictions).astype('uint8')
    
    #mask = tf.constant(mask, dtype=tf.int64)
    predictions = predictions.astype('uint8')
    semanctic_mask = np.where(np.isin(predictions, mask), 255, 0).astype('uint8')
    semanctic_mask = cv2.resize(semanctic_mask, (1280, 720), interpolation=cv2.INTER_NEAREST)
    kernel = np.ones((15,15), np.uint8)
    semanctic_mask = cv2.dilate(semanctic_mask, kernel, iterations = 4)
    #return predictions #, bool_mask.numpy()
    return colorized_preds, semanctic_mask


class segmentation:
    def __init__(self):
        # Params
        # decode_fn = Cityscapes.decode_target
        self.br = CvBridge()
        self.mask = []
        #parking['N', 'N', 'S', 'S', 'N', 'S', 'I', 'I', 'N', 'S', 'N', 'D', 'I', 'D', 'D', 'N', 'I', 'D', 'D']
        self.mask = [11]
        #TUM1
        #self.mask = [11,12,13,14,15,16,17,18]
        #self.mask = [4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]
        #TUM2
        
        self.model = network.modeling.__dict__['deeplabv3plus_mobilenet'](num_classes=19, output_stride=16)
        self.device = torch.device("cuda:0")
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('deeplabv3')
        checkpoint = torch.load(package_path + '/src/checkpoints/best_deeplabv3plus_mobilenet_cityscapes_os16.pth')
        self.model.load_state_dict(checkpoint["model_state"])
        self.model = nn.DataParallel(self.model)
        self.model.to(self.device)
        del checkpoint
        self.model.eval()
        


        # Publishers
        self.pub_color = rospy.Publisher('/semantic_color', Image, queue_size=10)
        self.pub = rospy.Publisher('/semantic_mask', Image, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/color/segmentation/image_raw", Image, self.callback)
        rospy.Subscriber("/bard/boolarray", CategorizationBoolArray, self.callback_mask)


    def callback(self, msg):
        image = self.br.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = np.float32(cv2.resize(image,(2048,1024))) / 255
        image = image.transpose(2, 0, 1)
        image = torch.from_numpy(image)
        image = torch.unsqueeze(image, 0)
        image.to(self.device)
        pre, semantic_mask = infer(self.model, image, self.mask)
        self.pub.publish(self.br.cv2_to_imgmsg(semantic_mask))
        self.pub_color.publish(self.br.cv2_to_imgmsg(pre))
        
    def callback_mask(self, msg):
        if len(msg.boolarray) == 19:
            self.mask = []
            for i, idx in enumerate(msg.boolarray):
                if not i:
                    self.mask.append(idx)
                
    

if __name__ == '__main__':
    rospy.init_node("semantic_mask_maker", anonymous=True)
    my_node = segmentation()
    rospy.spin()
