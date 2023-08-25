import numpy as np

cv = [[1,3,4,56,0,345],[2,3,2,56,87,255],[234,45,35,76,12,87]]
cv2 = [[1,6,4,56,0,345],[2,3,4,56,187,255],[234,45,35,0,12,87]]

output = np.true_divide(cv,cv2,where=(cv!=0) | (cv2!=0))
print(output)

def filter(arr):
  arr[~np.isfinite(arr)] = np.mean(arr[np.isfinite(arr)])
  return arr
print(filter(output))