import cv2

def draw_boxes(img, bbox, color="red", categories=None, names=None, offset=(0, 0)):
  img = img.copy()
  # print(bbox.shape)
  if color == "red":
    color = (0, 0, 255)
  else:
    color = (255, 255, 0)
  for i, box in enumerate(bbox):
    x1, y1, x2, y2 = [int(i) for i in box]
    x1 += offset[0]
    x2 += offset[0]
    y1 += offset[1]
    y2 += offset[1]
    # box text and bar

    # t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
    # cv2.rectangle(
    #     img, (x1, y1), (x1 + t_size[0] + 3, y1 + t_size[1] + 4), color, -1)
    # cv2.putText(img, label, (x1, y1 +
    #                          t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 2, [255, 255, 255], 2)
  return img