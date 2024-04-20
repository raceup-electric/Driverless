import numpy as np
import scipy.io
from scipy.interpolate import CubicSpline
from tph, look at gpt https://chat.openai.com/c/f9b98078-d431-480c-8f4a-3ba5ba5767de
#pip install quadprog

# Assume opt_min_curv function is already imported or defined here.

# Function to calculate normalized normal vectors
def calculate_normalized_normals(x, y):
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    magnitude = np.sqrt(dx_dt**2 + dy_dt**2)
    normals = np.vstack((-dy_dt, dx_dt)).T
    normalized_normals = normals / magnitude[:, np.newaxis]
    return normalized_normals

# Load the .mat file
mat_data = scipy.io.loadmat('workspace.mat')

# Assuming 'reftrack' is stored under the key 'xRTcc' and 'yRTcc', and track widths are constants
x = mat_data['xRTcc'].squeeze()
y = mat_data['yRTcc'].squeeze()
track_width_right = 3  # or load if available in .mat
track_width_left = 3   # or load if available in .mat

# Combine into a reftrack array [x, y, track_width_right, track_width_left]
reftrack = np.vstack((x, y, np.full_like(x, track_width_right), np.full_like(y, track_width_left))).T

# Calculate normal vectors
normvectors = calculate_normalized_normals(reftrack[:, 0], reftrack[:, 1])

# Parameters
kappa_bound = 0.2  # Maximum allowable curvature
w_veh = 2.0        # Vehicle width

# Run the optimization function
optimized_result = opt_min_curv(
    reftrack=reftrack,
    normvectors=normvectors,
    A=None,  # Assuming no A matrix is needed; adjust if necessary
    kappa_bound=kappa_bound,
    w_veh=w_veh,
    print_debug=True,
    plot_debug=True
)

# Print the optimized result
print("Optimized Track Coordinates:", optimized_result)
