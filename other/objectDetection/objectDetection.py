#!/usr/bin/env python3

import torchvision as Tv
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np

#TODO: run on the GPU

def get_prediction(img, threshold, model, COCO_INSTANCE_CATEGORY_NAMES):
      #img = Image.open(img_path).convert('RGB') # Load the image
      #img = img.convert('RGB')
      transform = Tv.transforms.Compose([Tv.transforms.ToTensor()]) # Defing PyTorch Transform
      img = transform(img) # Apply the transform to the image
      pred = model([img]) # Pass the image to the model
      pred_class = [COCO_INSTANCE_CATEGORY_NAMES[i] for i in list(pred[0]['labels'].numpy())] # Get the Prediction Score
      pred_boxes = [[(i[0], i[1]), (i[2], i[3])] for i in list(pred[0]['boxes'].detach().numpy())] # Bounding boxes
      pred_score = list(pred[0]['scores'].detach().numpy())
      pred_t = [pred_score.index(x) for x in pred_score if x > threshold][-1] # Get list of index with score greater than threshold.
      pred_boxes = pred_boxes[:pred_t+1]
      pred_class = pred_class[:pred_t+1]
      return pred_boxes, pred_class

#def object_detection_api(img, threshold=0.5, rect_th=3, text_size=3, text_th=3):

def object_detection_api(img, threshold=0.5):
      model = Tv.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
      model.eval()
      COCO_INSTANCE_CATEGORY_NAMES = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
                'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
                    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                        'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
                            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                                'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                                    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
                                        'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                                            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
                                                'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                                                    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
                                                        'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
                                                        ]
      boxes, pred_cls = get_prediction(img, threshold, model, COCO_INSTANCE_CATEGORY_NAMES) # Get predictions
      
      img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # Convert to RGB
      for i in range(len(boxes)):
            cv2.rectangle(img, boxes[i][0], boxes[i][1],color=(0, 255, 0), thickness=2) # Draw Rectangle with the coordinates
            cv2.putText(img,pred_cls[i], boxes[i][0],  cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),thickness=2) # Write the prediction class
      #TODO: return the image with the rectangles to compare with line
      plt.figure(figsize=(20,30)) # display the output image
      plt.imshow(img)
      plt.xticks([])
      plt.yticks([])
      plt.show()
      #reformat boxes into [20, ...] vs 2D array
      boxes = np.array(boxes)
      boxes = boxes.flatten()
      boxes = boxes.astype(int).tolist()
      return boxes, pred_cls

# torch.cuda.is_available();

# img = Image.open('./donut.png')
# object_detection_api(img, threshold=0.8)
# print("Done")
