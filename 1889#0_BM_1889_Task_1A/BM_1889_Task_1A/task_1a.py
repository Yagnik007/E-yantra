'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_1a.py
# Functions:		detect_shapes
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################





##############################################################

def detect_shapes(img):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns a nested list
    containing details of colored (non-white) shapes in that image

    Input Arguments:
    ---
    `img` :	[ numpy array ]
            numpy array of image returned by cv2 library

    Returns:
    ---
    `detected_shapes` : [ list ]
            nested list containing details of colored (non-white) 
            shapes present in image
    
    Example call:
    ---
    shapes = detect_shapes(img)
    """    
    detected_shapes = []

    ##############	ADD YOUR CODE HERE	##############
    #
    imggray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    _, threshold = cv2.threshold(imggray, 200, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        epsilon = 0.025*cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        cv2.drawContours(img, [approx], 0, (0), 3)
        x ,y, w , h = cv2.boundingRect(approx)
        if h >= 598 :
            continue
        rgb = np.array(cv2.mean(img2[y:y+h,x:x+w])).astype(np.uint8)
        colour = "Orange"
        if rgb[0]>150 and rgb[1]<150 :
            colour = "Red"
        elif rgb[1]>150 and rgb[0]<150 :
            colour = "Green"
        elif rgb[2]>150 :
            colour = "Blue"
        x,y = approx[0][0]
        M = cv2.moments(cnt)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        ls = []
        ls.append(colour)
        if len(approx) == 3:
            cv2.putText(img, "Triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, 0,2)
            cv2.circle(img, (cX, cY), 7, (0, 0, 0), -1)
            ls.append("Triangle")
            ls.append((cX,cY))
            
        elif len(approx) == 4:
            x ,y, w , h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            if aspectRatio >= 0.95 and aspectRatio <= 1.05:
                cv2.putText(img, "Square", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, 0,2)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                ls.append("Square")
                ls.append((cX,cY))
            else:
                cv2.putText(img, "Rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, 0,2)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                ls.append("Rectangle")
                ls.append((cX,cY))
        elif len(approx) == 5:
            cv2.putText(img, "Pentagon", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, 0,2)
            cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
            ls.append("Pentagon")
            ls.append((cX,cY))
        else:
            cv2.putText(img, "Circle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, 0,2)
            cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
            ls.append("Circle")
            ls.append((cX,cY))
        detected_shapes.append(ls)
        cv2.imshow("final", img)

    ##################################################
    
    return detected_shapes

def get_labeled_image(img, detected_shapes):
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
    """
    Purpose:
    ---
    This function takes the image and the detected shapes list as an argument
    and returns a labelled image

    Input Arguments:
    ---
    `img` :	[ numpy array ]
            numpy array of image returned by cv2 library

    `detected_shapes` : [ list ]
            nested list containing details of colored (non-white) 
            shapes present in image

    Returns:
    ---
    `img` :	[ numpy array ]
            labelled image
    
    Example call:
    ---
    img = get_labeled_image(img, detected_shapes)
    """
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

    for detected in detected_shapes:
        colour = detected[0]
        shape = detected[1]
        coordinates = detected[2]
        cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
    return img

if __name__ == '__main__':
    
    # path directory of images in 'test_images' folder
    img_dir_path = 'test_images/'

    # path to 'test_image_1.png' image file
    file_num = 1
    img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
    
    # read image using opencv
    img = cv2.imread(img_file_path)
    
    print('\n============================================')
    print('\nFor test_image_' + str(file_num) + '.png')
    
    # detect shape properties from image
    detected_shapes = detect_shapes(img)
    #print(detected_shapes)
    
    # display image with labeled shapes
    #img = get_labeled_image(img, detected_shapes)
    cv2.imshow("labeled_image", img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
    
    choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
    
    if choice == 'y':

        for file_num in range(1, 16):
            
            # path to test image file
            img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
            
            # read image using opencv
            img = cv2.imread(img_file_path)
    
            print('\n============================================')
            print('\nFor test_image_' + str(file_num) + '.png')
            
            # detect shape properties from image
            detected_shapes = detect_shapes(img)
            print(detected_shapes)
            
            # display image with labeled shapes
            img = get_labeled_image(img, detected_shapes)
            cv2.imshow("labeled_image", img)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()


