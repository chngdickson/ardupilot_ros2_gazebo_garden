import numpy as np
from typing import List
from kalmanFilterBetter import KalmanBoxTracker
import scipy.optimize as sci_optim

# Custom Modules for Type Hinting
class MatchedIndices(object):
  def __init__(self, det_idx:int, trk_idx:int):
    self.det_idx:int = det_idx
    self.trk_idx:int = trk_idx
    
class Sort_kf(object):
  def __init__(self, additional_params:int=0, max_undetected_times=1, min_hits=3, iou_threshold=0.3):
    """
    Parameters for SORT
      additional_params : Params other than len([x1,y1,x2,y2]), e.g. conf, cls, etc.
    """
    # Custom Params
    bounding_box = ['x1','y1','x2','y2']
    self.M = len(bounding_box)+ additional_params
    self.additional_params = additional_params
    self.has_additional_params = True if self.additional_params > 0 else False
    
    # Essentials of Hungarian Sorting algorithm
    self.max_age = max_undetected_times
    self.min_hits = min_hits
    self.iou_threshold = iou_threshold
    self.iou_threshold_repredict = iou_threshold/6
    self.trackers:List[KalmanBoxTracker] = []
    self.frame_count = 0
    
  def update(self, dets):
    """
    Parameters:
      dets : detections in the format [[x1,y1,x2,y2,additional_params],...]
    
    Returns:
      ret  : tracked objects in the format [[x1,y1,x2,y2,additional_params],...]
    
    """
    
    self.frame_count += 1
    # Local Params Initialization
    trks_pred = np.zeros((len(self.trackers),self.M))
    to_del = []
    ret = []
    
    # Get Predicted locations from existing trackers
    for i, trk in enumerate(self.trackers):
      pred_pos = trk.predict()
      if self.has_additional_params:
        trks_pred[i,:] = np.concatenate( (pred_pos, (self.additional_params)) )
      else:
        trks_pred[i,:] = pred_pos
      
    #   # 1. Register any deletion
    #   if np.any(np.isnan(pred_pos)):
    #     to_del.append(i)
    # # 2. Delete current trackers with NaN values
    # for t in reversed(to_del):
    #   self.trackers.pop(t)
    # # 3. Delete predicted trackers with NaN values
    # trks_pred = np.ma.compress_rows(np.ma.masked_invalid(trks_pred))

    # For trackers that are too close to each other, delete them.
    # I DONT delete trackers that Have NaN Values, I Simply repredict them
    to_del = self.NMS(trks_pred, overlapThresh=self.iou_threshold_repredict)
    for t in to_del:
      self.trackers.pop(t)
    trks_pred = np.delete(trks_pred, to_del, axis=0)
    
    # Hungarian Algorithm and IOU
    matched, unmatched_dets = self.get_matches_and_unmatched_detections(dets, trks_pred, self.iou_threshold)
    
    # Update matched trackers with assigned detections
    for m in matched:
      # print("matched_dets",len(matched),m.det_idx, dets.shape)
      det_bbox = dets[m.det_idx,0:4]
      additional_params = dets[m.det_idx,4:] if self.has_additional_params else None
      self.trackers[m.trk_idx].update(det_bbox,additional_params, update_ori_bbox=True)
    
    
    # For unmatched detections, Initialize new KalmanFilter trackers 
    for i in unmatched_dets:
      # print("unmatched_dets",i, dets.shape)
      det_bbox = dets[i,0:4]
      additional_params = dets[i,4:] if self.has_additional_params else None
      self.trackers.append(KalmanBoxTracker(det_bbox, additional_params))
    
    i = len(self.trackers)
    for trk in reversed(self.trackers):
      d = trk.get_state()
      if (trk.hit_streak >= self.min_hits):
        ret.append(d)
      i -= 1
      
      # 4. Delete current trackers with that is too old.
      if(trk.time_since_update > self.max_age):
        # print("Deleted obj")
        self.trackers.pop(i)
    if(len(ret)>0):
      return np.vstack(ret)
    return np.empty((0,self.M))
  
  """
  Beyond these are Helper functions
  """
  def iou_batch(self, bb_det, bb_trg):
    """
    Almost like a NMS function
    Compare Bounding boxes between two batches; Detected and Target
    
    Parameters:
      bb_det  : [num_preds,4] = [[x1,y1,x2,y2], ...]]
      bb_trg  : [num_trg  ,4] = [[x1,y1,x2,y2], ...]]
    
    Returns:
      iou_mat : [num_det, num_trg] an array of Intersection over Union Matrix
                just like a similarity matrix.
    """
    bb_det = np.expand_dims(bb_det, 1).astype(np.int32)
    bb_trg = np.expand_dims(bb_trg, 0).astype(np.int32)
    
    xx1 = np.maximum(bb_trg[...,0], bb_det[..., 0])
    yy1 = np.maximum(bb_trg[..., 1], bb_det[..., 1])
    xx2 = np.minimum(bb_trg[..., 2], bb_det[..., 2])
    yy2 = np.minimum(bb_trg[..., 3], bb_det[..., 3])
    w = np.maximum(0., xx2 - xx1 )
    h = np.maximum(0., yy2 - yy1 )
    wh = w * h
    area_batch = ((bb_trg[..., 2] - bb_trg[..., 0]) * (bb_trg[..., 3] - bb_trg[..., 1]) +\
                  (bb_det[..., 2] - bb_det[..., 0]) * (bb_det[..., 3] - bb_det[..., 1]) - wh)
    return wh / area_batch
    
  def get_matches_and_unmatched_detections(self, dets, trks, iou_threshold=0.3) \
    -> tuple[List[MatchedIndices], List[int]]:
    """
    Get matches and unmatched detections between Detected and Tracked Bounding Boxes
    
    Parameters:
      dets : [N, 4] = [[x1,y1,x2,y2, ...]
      trks : [N, 4] = [[x1,y1,x2,y2], ...]
      
    Returns:
      matched_indices      : [N, 2] = [[det_idx, trk_idx], ...]
      unmatched_detections : [N, 1] = [[det_idx], ...]
    There is a 3rd variable called unmatched_trackers, but since 
    it's are already tracking it in sort, we don't need to return it.
    """
    # Initialize Variables
    matches_idxs: List[MatchedIndices] = []
    unmatched_detections: List[int]    = []
    # print("DET and Trks shape",dets.shape, trks.shape)
    if len(trks) == 0:
      return matches_idxs, np.arange(len(dets)).tolist()
    
    iou_matrix = self.iou_batch(dets, trks)
    
    if iou_matrix.shape[0] > 0 and iou_matrix.shape[1] > 0:
      # If only 2 bounding box is Similar and exceeded the threshold, find their indices
      a = (iou_matrix > iou_threshold).astype(np.int32)
      # if np.sum(a, axis=0).max() == 1 and np.sum(a,axis=1).max() == 1:
      #   # print("stack dude did it")
      #   matched_indices = np.stack(np.where(a), axis=1)
      # else: # use Hungarian Algorithm
      #   # print("Hungarian Algorithm")
      det_idx,trk_idx = sci_optim.linear_sum_assignment(-iou_matrix)
      matched_indices = np.array(list(zip(det_idx,trk_idx)))
      for det_idx, trk_idx in matched_indices:
        if det_idx <0 or trk_idx < 0:
          print("Failed, Some idices are less than 0")
    else:
      matched_indices = np.empty((0,2))

    # Get unmatched detections
    for det_idx, det in enumerate(dets):
      # print(matched_indices)
      # print("Tuple indices MUST BE",det_idx, matched_indices[:,0])
      if(det_idx not in matched_indices[:,0]):
        unmatched_detections.append(det_idx)
    
    # Filter out matched detections with low IOU
    for m in matched_indices:
      # print("iou",m)
      # print(iou_matrix.shape, len(matched_indices))
      if (iou_matrix[m[0],m[1]] < iou_threshold):
        unmatched_detections.append(m[0])
      else:
        matches_idxs.append(MatchedIndices(det_idx=m[0],trk_idx=m[1]))
    

    return matches_idxs, unmatched_detections
  
  def NMS(self, boxes, overlapThresh = 0.4)-> List[int]:
    # Return an empty list, if no boxes given
    if len(boxes) == 0:
        return []
    x1 = boxes[:, 0]  # x coordinate of the top-left corner
    y1 = boxes[:, 1]  # y coordinate of the top-left corner
    x2 = boxes[:, 2]  # x coordinate of the bottom-right corner
    y2 = boxes[:, 3]  # y coordinate of the bottom-right corner
    # Compute the area of the bounding boxes and sort the bounding
    # Boxes by the bottom-right y-coordinate of the bounding box
    areas = (x2 - x1 + 1) * (y2 - y1 + 1) # We add 1, because the pixel at the start as well as at the end counts
    # The indices of all boxes at start. We will redundant indices one by one.
    indices = np.arange(len(x1))
    for i,box in enumerate(boxes):
      # Create temporary indices  
      temp_indices = indices[indices!=i]
      # Find out the coordinates of the intersection box
      xx1 = np.maximum(box[0], boxes[temp_indices,0])
      yy1 = np.maximum(box[1], boxes[temp_indices,1])
      xx2 = np.minimum(box[2], boxes[temp_indices,2])
      yy2 = np.minimum(box[3], boxes[temp_indices,3])
      # Find out the width and the height of the intersection box
      w = np.maximum(0, xx2 - xx1 + 1)
      h = np.maximum(0, yy2 - yy1 + 1)
      # compute the ratio of overlap
      overlap = (w * h) / areas[temp_indices]
      # if the actual boungding box has an overlap bigger than treshold with any other box, remove it's index  
      if np.any(overlap) > overlapThresh:
        indices = indices[indices != i]
    #return only the boxes at the remaining indices
    
    to_del = []
    for i in range(len(boxes)):
      if i not in indices:
        to_del.append(i)
    to_del = sorted(to_del, reverse=True)
    return to_del