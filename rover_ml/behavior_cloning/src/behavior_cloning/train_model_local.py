#!/usr/bin/env python
# -*- coding: utf-8 -*-
#https://github.com/juano2310/CarND-Behavioral-Cloning-P3-Juan/blob/master/model.py
#https://github.com/udacity/self-driving-car/blob/master/steering-models/community-models/rambo/train.py

import os
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from keras import backend as K
from keras.models import Model, Sequential
from keras.layers import Dense, GlobalAveragePooling2D, MaxPooling2D, Lambda, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.core import Flatten, Dense, Dropout, SpatialDropout2D
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint
from keras.callbacks import EarlyStopping

import sklearn
from sklearn.model_selection import train_test_split

path = '/content/drive/My Drive/research/diy_driverless_car_ROS/rover_ml/output'
data_set = 'office_2'
tar_file = data_set + ".tar.gz"

data_set = 'office_2'

samples = []
with open('../../../output/' + data_set + '/interpolated.csv') as csvfile:
     reader = csv.reader(csvfile)
     for line in reader:
         samples.append(line)

sklearn.utils.shuffle(samples)
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

print("Number of traing samples: ",len(train_samples))
print("Number of validation samples: ",len(validation_samples))

#index,timestamp,width,height,frame_id,filename,angle,speed
def generator(samples, batch_size=32, aug=0):
     num_samples = len(samples)

     while 1: # Loop forever so the generator never terminates
         for offset in range(0, num_samples, batch_size):
             batch_samples = samples[offset:offset+batch_size]

             #print(batch_samples)
             images = []
             angles = []
             for batch_sample in batch_samples:
                 if batch_sample[5] != "filename":
                     path = os.path.normpath(batch_sample[5]).split(os.path.sep) 
                     name = '../../../output/office_2/center/'+path[1].split('\\')[-1]    
                     center_image = cv2.imread(name)
                     center_image = cv2.resize(center_image, (320,180)) #resize from 720x1280 to 180x320
                     center_image = center_image/255.
                     #plt.imshow(left_image)
                     #plt.show()                 
                     angle = float(batch_sample[6])
                     images.append(center_image)
                     angles.append(angle)
                     if aug:
                       flip_image = np.fliplr(center_image)
                       flip_image = flip_image/255.
                       flip_angle = -1 * angle
                       images.append(flip_image)
                       angles.append(flip_angle)
                 
             X_train = np.array(images)
             y_train = np.array(angles)
             
             yield sklearn.utils.shuffle(X_train, y_train)

# compile and train the model using the generator function
batch_size_value = 32
n_epoch = 30

train_generator = generator(train_samples, batch_size=batch_size_value, aug=1)
validation_generator = generator(validation_samples, batch_size=batch_size_value, aug=0)

model = Sequential()

# trim image to only see section with road
model.add(Cropping2D(cropping=((75,20), (0,0)), input_shape=(180,320,3)))

# Preprocess incoming data, centered around zero with small standard deviation
#model.add(Lambda(lambda x: (x / 255.0) - 0.5))

#Nvidia model
model.add(Convolution2D(24, (5, 5), activation="relu", name="conv_1", strides=(2, 2)))
model.add(Convolution2D(36, (5, 5), activation="relu", name="conv_2", strides=(2, 2)))
model.add(Convolution2D(48, (5, 5), activation="relu", name="conv_3", strides=(2, 2)))
model.add(SpatialDropout2D(.5, dim_ordering='default'))

model.add(Convolution2D(64, (3, 3), activation="relu", name="conv_4", strides=(1, 1)))
model.add(Convolution2D(64, (3, 3), activation="relu", name="conv_5", strides=(1, 1)))

model.add(Flatten())

model.add(Dense(1164))
model.add(Dropout(.5))
model.add(Dense(100, activation='relu'))
model.add(Dropout(.5))
model.add(Dense(50, activation='relu'))
model.add(Dropout(.5))
model.add(Dense(10, activation='relu'))
model.add(Dropout(.5))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')
model.summary()

# checkpoint
filepath="../../weights/weights-improvement-{epoch:02d}-{val_loss:.2f}.hdf5"
checkpoint = ModelCheckpoint(filepath, monitor='val_loss', verbose=1, save_best_only=True, mode='auto', period=1)

early_stop = EarlyStopping(monitor='val_loss', patience=10)

callbacks_list = [checkpoint, early_stop]

# Fit the model
history_object = model.fit_generator(train_generator, steps_per_epoch=(len(train_samples) / batch_size_value), validation_data=validation_generator, validation_steps=(len(validation_samples)/batch_size_value), callbacks=callbacks_list, epochs=n_epoch)

# Save model
model.save('model.h5')
with open('model.json', 'w') as output_json:
    output_json.write(model.to_json())

# Save TensorFlow model
tf.train.write_graph(K.get_session().graph.as_graph_def(), logdir='.', name='model.pb', as_text=False)

# Plot the training and validation loss for each epoch
print('Generating loss chart...')
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.savefig('model.png')

# Done
print('Done.')










