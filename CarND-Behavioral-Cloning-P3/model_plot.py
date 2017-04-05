from keras.utils import plot_model
from keras.models import Sequential
from keras.layers.convolutional import Conv2D, Cropping2D
from keras.layers.core import Flatten, Dense, Lambda, Dropout

ROW, COL, CH = 160, 320, 3
# Model
MODEL = Sequential()
MODEL.add(Lambda(lambda x: x / 127.5 - 1.0, input_shape=(ROW, COL, CH)))
MODEL.add(Cropping2D(cropping=((55, 20), (0, 0))))
MODEL.add(Conv2D(24, 5, strides=(2, 2), activation='relu'))
MODEL.add(Dropout(0.8))
MODEL.add(Conv2D(36, 5, strides=(2, 2), activation='relu'))
MODEL.add(Conv2D(48, 5, strides=(2, 2), activation='relu'))
MODEL.add(Conv2D(64, 3, activation='relu'))
MODEL.add(Conv2D(64, 3, activation='relu'))
MODEL.add(Flatten())
MODEL.add(Dense(100))
MODEL.add(Dense(50))
MODEL.add(Dense(10))
MODEL.add(Dense(1))

plot_model(MODEL, to_file='model.png')