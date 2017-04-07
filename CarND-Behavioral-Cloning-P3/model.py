"""
The script used to create and train the model.
"""

import csv
import cv2
import numpy as np

from keras.models import Sequential
from keras.layers.convolutional import Conv2D, Cropping2D
from keras.layers.core import Flatten, Dense, Lambda, Dropout

from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

# load data
LINES = []
CORRECTION = 0.2

with open('./data/driving_log.csv') as csvfile:
    READER = csv.reader(csvfile)
    for line in READER:
        LINES.append(line)

TRAIN_SAMPLES, VALIDATION_SAMPLES = train_test_split(LINES, test_size=0.2)

def generator(samples, batch_size=32):
    """
    generat training samples
    """
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

        images = []
        measurements = []
        for batch_sample in batch_samples:
            # Use 3 cameras
            measurement = float(batch_sample[3])
            measurement_left = measurement + CORRECTION
            measurement_right = measurement - CORRECTION

            for i in range(3):
                source_path = batch_sample[i]
                filename = source_path.split('/')[-1]
                current_path = './data/IMG/' + filename
                image_bgr = cv2.imread(current_path)
                image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                # image = cv2.resize(image, (80, 160), cv2.INTER_NEAREST)
                images.append(image)
                # Augment image
                images.append(cv2.flip(image, 1))

            measurements.extend([measurement,
                                 measurement*-1,
                                 measurement_left,
                                 measurement_left*-1,
                                 measurement_right,
                                 measurement_right*-1])


            x_train = np.array(images)
            y_train = np.array(measurements)

            yield shuffle(x_train, y_train)

TRAIN_GENERATOR = generator(TRAIN_SAMPLES, batch_size=32)
VALIDATION_GENERATOR = generator(VALIDATION_SAMPLES, batch_size=32)

ROW, COL, CH = 160, 320, 3
# Model
MODEL = Sequential()
MODEL.add(Lambda(lambda x: x / 127.5 - 1.0, input_shape=(ROW, COL, CH)))
MODEL.add(Cropping2D(cropping=((60, 20), (0, 0))))
MODEL.add(Conv2D(24, 5, strides=(2, 2), activation='relu'))
MODEL.add(Dropout(0.7))
MODEL.add(Conv2D(36, 5, strides=(2, 2), activation='relu'))
MODEL.add(Conv2D(48, 5, strides=(2, 2), activation='relu'))
MODEL.add(Conv2D(64, 3, activation='relu'))
MODEL.add(Conv2D(64, 3, activation='relu'))
# MODEL.add(Dropout(0.8))
MODEL.add(Flatten())
MODEL.add(Dense(100))
MODEL.add(Dense(50))
MODEL.add(Dense(10))
MODEL.add(Dense(1))


MODEL.compile(loss='mse', optimizer='adam')
MODEL.fit_generator(TRAIN_GENERATOR,
                    steps_per_epoch=len(TRAIN_SAMPLES),
                    validation_data=VALIDATION_GENERATOR,
                    validation_steps=len(VALIDATION_SAMPLES),
                    epochs=3)

# Save model
MODEL.save('model.h5')
