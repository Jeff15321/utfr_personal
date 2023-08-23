import numpy as np
import cv2
#import xgboost

def classical_process(left_img_, right_img_, translation, intrinsics_left, cone_classifier, xgb_model, yellow_bgr, orange_bgr, blue_bgr): #add detection_method into parameters
    
    #TODO - move to different file
    #TODO - package in custom datatype/msg utfr_msgs_conedetections
    #TODO - baseline, f make parameters in loadParams

    baseline = abs(translation[0])/10 #distance between cameras
    f = intrinsics_left[0][0] #focal length in pixels from camera matrix
    colours = ['blue', 'yellow', 'orange']

    
    #cone detection
    
    results_left = cone_classifier.detectMultiScale(left_img_, 1.2, 4)
    results_right = cone_classifier.detectMultiScale(right_img_, 1.2, 4)

    left_width_ = int(left_img_.shape[1])

    #detections array
    if (len(results_left) > 0):
      coneDetections = np.zeros((np.shape(results_left)[0],4))
      
      i = 0
      for (x, y, w, h) in results_left:
        
        if (y <= 470):
          continue

        #colour detection

        #TODO - import color detection xgboost classifier
        color = labelColour(left_img_, x, y, w, h, xgb_model, colours, yellow_bgr, orange_bgr, blue_bgr)

        #stereo matching and depth

        right_x, right_y = find_cone_other_frame(left_img_, left_width_, x, y, w, h, 20, right_img_)
        right_x_bot_centre = right_x + int(w/2)
        right_y_bot_centre = right_y + h

        left_x_bot_centre = x + int(w/2)
        left_y_bot_centre = y + h
        
        disparity = abs(left_x_bot_centre - right_x_bot_centre)

        depth = find_depth((right_x_bot_centre, right_y_bot_centre), (left_x_bot_centre, left_y_bot_centre), right_img_, left_img_, baseline, f)

        #3d coordinate mapping
        
        f_u = intrinsics_left[0][0]
        f_v = intrinsics_left[1][1]
        c_u = int((np.shape(left_img_)[1])/2)
        c_v = int((np.shape(left_img_)[0])/2)

        coneDetections[i][0], coneDetections[i][1], coneDetections[i][2] = project3DPoints(((x+int(w/2)), y+int(h/2)), f_u, f_v, \
          c_u, c_v, disparity, baseline, depth)  

        coneDetections[i][3] = color

        i += 1

      return results_left, results_right, coneDetections

    #if no detections, return empty list
    return results_left, results_right, []


def find_cone_other_frame(initialFrame, frameWidth, fX, fY, fW, fH, uncertainty, otherFrame):
    search_area = crop_image(0, fY-uncertainty, frameWidth, fH+2*uncertainty, otherFrame)
    template_im = crop_image(fX, fY, fW, fH, initialFrame)
    result = cv2.matchTemplate(search_area, template_im, 0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    top_left = min_loc
    result_x = top_left[0]
    result_y = top_left[1] + fY - uncertainty
    #top left coordinates of other frame, x, y
    return result_x, result_y


def find_depth(right_point, left_point, frame_right, frame_left, baseline, f_pixel):
    
    # Convert focal length, f, from [mm] to [pixel]
    # TODO: WITH NEW CAMERA CALIBRATION, DIRECTLY INPUT PIXEL VALUES
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape

    x_right = right_point[0]
    x_left = left_point[0]

    # Calculate the disparity
    disparity = abs(x_left-x_right) #Displacement between left and right frames [pixels]

    # Calculate depth, z
    zDepth = (baseline*f_pixel)/disparity #Depth in [cm]
    

    return zDepth

def project3DPoints(pointLeft, f_u, f_v, c_u, c_v, disparity, baseline, depth):
    u_l, v_l = pointLeft[0], pointLeft[1]

    #coordinates in [m]

    x = ((baseline/disparity) * (u_l - c_u))/100
    y = ((baseline/disparity) * (f_u/f_v) * (v_l - c_v))/100
    z = depth/100

    return (x, y, z)

def crop_image(fX, fY, fW, fH, image_in):
    image_out = image_in[fY:fY+fH, fX:fX + fW]
    return image_out

def crop_middle(image):
    w = image.shape[0]

    c_im = image[:, int(w/4):int(w/4)*3]
    return c_im

yellow_bgr = np.array([50, 170, 225])
blue_bgr = np.array([200, 90, 40])
orange_bgr = np.array([30, 70, 255])

def euclidean_distance(v1, v2):
    return np.sqrt((v1[0] - v2[0])*(v1[0] - v2[0]) + (v1[1] - v2[1])*(v1[1] - v2[1]) + (v1[2] - v2[2])*(v1[2] - v2[2]))

def labelColour(image, x, y, w, h, xgb_model, colours, yellow_bgr, orange_bgr, blue_bgr):
    
    im_c = image[y:y+h, x:x+w]
    im_c = crop_middle(im_c)

    mean = cv2.mean(im_c)[:3]
    mean = [[mean[2], mean[1], mean[0]]]
    rgb_value = np.array(mean, dtype=object)

    xgb_pred = xgb_model.predict(rgb_value)

    colour_pred = colours[int(xgb_pred)]

    if colour_pred == 'blue':
      return 1
    elif colour_pred == 'yellow':
      return 2
    elif colour_pred == 'orange':
      return 3
    else:
      return 0