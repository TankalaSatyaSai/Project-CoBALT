import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Input

# Load data from CSV files
lidar_data = pd.read_csv('lidar_data.csv', header=None)  
cmd_vel_data = pd.read_csv('cmd_vel_data.csv', header=None) 

model = Sequential([
    Dense(64, activation='relu', input_shape=(lidar_data.shape[1],)),  # Adjust input shape
    Dense(32, activation='relu'),
    Dense(2)  # Assuming 2 outputs for cmd_vel_data
])

# Compile the model
model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])

# Train the model
model.fit(lidar_data, cmd_vel_data, epochs=10, batch_size=32, validation_split=0.2)
model.save('trained_model.h5')

