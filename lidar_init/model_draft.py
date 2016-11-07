# resnet50

from keras.applications.resnet50 import ResNet50
from keras.preprocessing import image
from keras.applications.resnet50 import preprocess_input, decode_predictions
import numpy as np
import tensorflow as tf
tf.python.control_flow_ops = tf

model = ResNet50(weights='imagenet')

import cv2
# cap = cv2.VideoCapture("Off-Road Car Action From the Desert _ Dakar Rally 2016-ZpMVhhXAy98.mkv")
cap = cv2.VideoCapture("/home/bowen/Desktop/Sample Videos/2016-03-01 14-18-38-542378_Partition_0.avi")

while True:
	if cap.grab():
		flag, frame = cap.retrieve()
		if not flag:
			continue
		else:
			newframe = cv2.resize(frame,(224,224))
			x = image.img_to_array(newframe)
			x = np.expand_dims(x, axis=0)
			x = preprocess_input(x)
			preds = model.predict(x)
			# decode the results into a list of tuples (class, description, probability)
			# (one such list for each sample in the batch)
			print('Predicted:', decode_predictions(preds))
			cv2.imshow('video', frame)
	if cv2.waitKey(10) == 27:
		break






img_path = 'elephant.jpg'
img = image.load_img(img_path, target_size=(224, 224))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
x = preprocess_input(x)
preds = model.predict(x)
# decode the results into a list of tuples (class, description, probability)
# (one such list for each sample in the batch)
print('Predicted:', decode_predictions(preds, top=3)[0])
# Predicted: [(u'n02504013', u'Indian_elephant', 0.82658225), (u'n01871265', u'tusker', 0.1122357), (u'n02504458', u'African_elephant', 0.061040461)]



