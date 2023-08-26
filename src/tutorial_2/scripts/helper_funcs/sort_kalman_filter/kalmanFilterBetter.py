import filterpy.kalman as kalman
import numpy as np

class KalmanBoxTracker(object):
  """
  This class represents the internal state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self, bbox, additional_infos):
    """
    Initialize a tracker using initial bounding box
    
    Parameters:
      bbox : [x1,y1,x2,y2]
      additional_infos : np.array([info1, info2, ...]) 
    
    """
    assert len(bbox) == 4, f"bbox must be in the form [x1,y1,x2,y2], len bbox is {len(bbox)}"
    self.kf = kalman.KalmanFilter(dim_x=7, dim_z=4)
    self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],[0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
    self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

    self.kf.R[2:,2:] *= 10. # R: Covariance matrix of measurement noise (set to high for noisy inputs -> more 'inertia' of boxes')
    self.kf.P[4:,4:] *= 1. #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    self.kf.Q[-1,-1] *= 0.2 # Q: Covariance matrix of process noise (set to high for erratically moving things)
    self.kf.Q[4:,4:] *= 0.5
    self.kf.x[:4] = self.convert_bbox_to_z(bbox) # STATE VECTOR
    
    # Initialize Tracking Params
    self.time_since_update = 0
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.hits = 0
    self.hit_streak = 0
    self.age = 0
    
    # Initialize Additional Infos
    # Because there is no Pred_bbox yet, we use the bbox as the pred_bbox
    self.additional_infos = additional_infos
    self.pred_bbox  = bbox
    
    # This is for the case when we want to update the original bbox from YOLO
    # to do a comparison for the pred bbox from Kalman Filter
    self.original_bbox = bbox
    self.ori_count = 0
      
  def update(self, bbox, additional_infos, update_ori_bbox=False):
    """
    Updates the state vector with observed bbox
    Sometimes it updates itself with it's previous predicted bbox, to better train kalman filter instead of deleting nans.
    
    Parameters:
      bbox : [x1,y1,x2,y2]
      additional_infos : np.array([info1, info2, ...])
    
    Returns:
      None
    """
    assert len(bbox) == 4, f"bbox must be in the form [x1,y1,x2,y2], len bbox is {len(bbox)}"
    self.time_since_update = 0
    self.hits += 1
    self.hit_streak += 1
    self.pred_bbox = bbox
    
    if update_ori_bbox:
      self.original_bbox = bbox
      self.additional_infos = additional_infos
      self.ori_count = self.count
    self.kf.update(self.convert_bbox_to_z(bbox))
    
  def predict(self):
    """
    Returns:
      pred_bbox : [x1,y1,x2,y2]
    """
    # Update timings.
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    
    # Perform correction to Kalman Filter, to tackle negative values
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    
    # Perform prediction
    self.kf.predict()
    curr_pred_bbox = self.convert_x_to_bbox(self.kf.x)
    if np.any(curr_pred_bbox<0) or np.any(np.isnan(curr_pred_bbox)):
      self.update(self.pred_bbox, self.additional_infos, update_ori_bbox=False)
      return self.pred_bbox
    else:
      return curr_pred_bbox
  
  def get_state(self):
    """
    Motivation:
      This function is separated to predict because, 
      we want to save memory by not saving Huge amount of additional_infos
    Returns:
      state : [x1,y1,x2,y2] + [additional_infos]
    """
    if self.additional_infos != None:
      # TODO: Update additional Params to include comparison for segmentation.
      return np.concatenate(self.pred_bbox, self.additional_infos)
    else:
      return self.pred_bbox
    
  def convert_bbox_to_z(self,bbox):
    """
    Parameters:
      bbox : [x1,y1,x2,y2]
    Returns:
      z : 
      
      [[xc],
      [yc],
      [s],
      [r]] 
      where xc,yc is the centre of the box and s is the scale/area and r is the aspect ratio
    """
    # print("bbox",bbox)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    x = bbox[0] + w/2.
    y = bbox[1] + h/2.
    s = w * h    #scale is just area
    r = w / float(h)
    return np.array([x, y, s, r],dtype=float).reshape((4, 1))

  def convert_x_to_bbox(self, x):
    """
    Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
    [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
    """
    # print("States", x.shape)
    w = np.sqrt(x[2] * x[3])
    h = x[2] / w
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.], dtype=float).flatten()