#!/usr/bin/env python3

import torchvision as Tv
#hacky soln, TODO:figure out pyenv next
#ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
#if ros_path in sys.path:
#    sys.path.remove()
import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import matplotlib.pyplot as plt
from PIL import Image

#TODO: run on the GPU

def get_prediction(img_path, threshold):
      img = Image.open(img_path).convert('RGB') # Load the image
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


def object_detection_api(img_path, threshold=0.5, rect_th=3, text_size=3, text_th=3):

      boxes, pred_cls = get_prediction(img_path, threshold) # Get predictions
      return boxes
      img = cv2.imread(img_path) # Read image with cv2
      img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # Convert to RGB
      for i in range(len(boxes)):
            cv2.rectangle(img, boxes[i][0], boxes[i][1],color=(0, 255, 0), thickness=rect_th) # Draw Rectangle with the coordinates
            cv2.putText(img,pred_cls[i], boxes[i][0],  cv2.FONT_HERSHEY_SIMPLEX, text_size, (0,255,0),thickness=text_th) # Write the prediction class
      #TODO: return the image with the rectangles to compare with line
      plt.figure(figsize=(20,30)) # display the output image
      plt.imshow(img)
      plt.xticks([])
      plt.yticks([])
      plt.show()

# torch.cuda.is_available();
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
object_detection_api('./donut.png', threshold=0.8)
print("Done")
