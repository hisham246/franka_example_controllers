import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load the data
actual_cartesian_state = pd.read_csv('results/actual_cartesian_state.csv')
desired_cartesian_state = pd.read_csv('results/desired_cartesian_state.csv')
actual_stiffness_state = pd.read_csv('results/actual_stiffness_state.csv')
desired_stiffness_state = pd.read_csv('results/desired_stiffness_state.csv')

# Find the minimum number of rows among the files
min_length = min(len(actual_cartesian_state), len(desired_cartesian_state), len(actual_stiffness_state), len(desired_stiffness_state))

# Truncate the dataframes to the minimum length
actual_cartesian_state = actual_cartesian_state.iloc[:min_length]
desired_cartesian_state = desired_cartesian_state.iloc[:min_length]
actual_stiffness_state = actual_stiffness_state.iloc[:min_length]
desired_stiffness_state = desired_stiffness_state.iloc[:min_length]

# Calculate Cartesian state errors
cartesian_errors = pd.DataFrame()
cartesian_errors['time'] = desired_cartesian_state['time'].values
cartesian_errors['x'] = (desired_cartesian_state['x'].values - actual_cartesian_state['x'].values)
cartesian_errors['y'] = (desired_cartesian_state['y'].values - actual_cartesian_state['y'].values)
cartesian_errors['z'] = (desired_cartesian_state['z'].values - actual_cartesian_state['z'].values)
cartesian_errors['roll'] = (desired_cartesian_state['roll'].values - actual_cartesian_state['roll'].values)
cartesian_errors['pitch'] = (desired_cartesian_state['pitch'].values - actual_cartesian_state['pitch'].values)
cartesian_errors['yaw'] = (desired_cartesian_state['yaw'].values - actual_cartesian_state['yaw'].values)

# Calculate Stiffness state errors
stiffness_errors = pd.DataFrame()
stiffness_errors['time'] = desired_stiffness_state['time'].values
stiffness_errors['translational_stiffness'] = (desired_stiffness_state[['tx', 'ty', 'tz']].mean(axis=1).values - actual_stiffness_state[['tx', 'ty', 'tz']].mean(axis=1).values)
stiffness_errors['rotational_stiffness'] = (desired_stiffness_state[['rx', 'ry', 'rz']].mean(axis=1).values - actual_stiffness_state[['rx', 'ry', 'rz']].mean(axis=1).values)

# Plot settings
plt.rc('font', family='serif')
plt.rc('text', usetex=False)
plt.rc('xtick', labelsize=18)  # Increased tick label size
plt.rc('ytick', labelsize=18)  # Increased tick label size
plt.rc('axes', labelsize=18)  # Increased label size

# Plot Cartesian state errors
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
axs[0].plot(cartesian_errors['time'].values, cartesian_errors['x'].values, label='x error')
axs[0].plot(cartesian_errors['time'].values, cartesian_errors['y'].values, label='y error')
axs[0].plot(cartesian_errors['time'].values, cartesian_errors['z'].values, label='z error')
axs[0].set_ylabel('Position Error (m)', fontsize=18)
axs[0].set_title('Cartesian Position Errors', fontsize=18)
axs[0].legend(fontsize=14)
axs[0].grid(True)

axs[1].plot(cartesian_errors['time'].values, cartesian_errors['roll'].values, label='roll error')
axs[1].plot(cartesian_errors['time'].values, cartesian_errors['pitch'].values, label='pitch error')
axs[1].plot(cartesian_errors['time'].values, cartesian_errors['yaw'].values, label='yaw error')
axs[1].set_xlabel('Time (s)', fontsize=18)
axs[1].set_ylabel('Orientation Error (rad)', fontsize=18)
axs[1].set_title('Cartesian Orientation Errors', fontsize=18)
axs[1].legend(fontsize=14)
axs[1].grid(True)
fig.tight_layout()
fig.patch.set_facecolor('white')
plt.show()

# Plot Stiffness state errors
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
axs[0].plot(stiffness_errors['time'].values, stiffness_errors['translational_stiffness'].values, label='Translational Stiffness Error')
axs[0].set_ylabel('Error', fontsize=18)
axs[0].set_title('Translational Stiffness Errors', fontsize=18)
axs[0].legend(fontsize=14)
axs[0].grid(True)

axs[1].plot(stiffness_errors['time'].values, stiffness_errors['rotational_stiffness'].values, label='Rotational Stiffness Error')
axs[1].set_xlabel('Time (s)', fontsize=18)
axs[1].set_ylabel('Error', fontsize=18)
axs[1].set_title('Rotational Stiffness Errors', fontsize=18)
axs[1].legend(fontsize=14)
axs[1].grid(True)
fig.tight_layout()
fig.patch.set_facecolor('white')
plt.show()

