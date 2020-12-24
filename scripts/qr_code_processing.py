#!/usr/bin/env python
# coding=utf8

import numpy as np
import cv2
import os.path
import zbar
from PIL import Image,ImageColor
import copy
from scipy.spatial import distance as dist

class QrCodeEstimation():
    def __init__(self, _threshold = 500, _size_image = 0., show_image = False):


        # initialization
        self.input_mode = False
        self.show_image = show_image
        self.threshold = _threshold

        self.box_pts = []
        self.type = ""
        self.data = ""
        self.frame = None
        self.img_object = None

        self.size_image = _size_image

        self.max_dist = 10.0
        self.camera_parameters = np.array(
            [[653.564007, 0.000000, 326.174970], [0.000000, 655.878899, 247.761618], [0, 0, 1]])
        self.camera_distortion_param = np.array([-0.426801, 0.249082, -0.002705, -0.001600, 0.000000])

    def norm_2d_points(self, pts):
        # sort the points based on their x-coordinates
        xSorted = pts[np.argsort(pts[:, 0]), :]
        # grab the left-most and right-most points from the sorted
        # x-roodinate points
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost
        # now that we have the top-left coordinate, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr) = rightMost[np.argsort(D)[::-1], :]
        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        return np.array([tl,bl, br,tr], dtype="float32")

    def update(self, frame):
        self.frame = frame

        blur_state, val_blur = self.is_blur(frame)
        if blur_state:
            print("image blur: %d" %val_blur)
            return frame, None, None, None


        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, dstCn=0)
        pil = Image.fromarray(gray)
        width, height = pil.size
        raw = pil.tobytes()
        image = zbar.Image(width, height, 'Y800', raw)

        scanner.scan(image)

        translation = None
        rotation_rad = None

        for symbol in image:
            print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data
            self.data = symbol.data
            points_2d = np.array([item for item in symbol.location], dtype="double")
            points_2d = self.norm_2d_points(points_2d)
            object_points_3d = self.output_3d_points(self.size_image)



            rotation_rad, translation = self.iterative_solve_pnp(object_points_3d, points_2d,
                                                                 self.camera_parameters,
                                                                 self.camera_distortion_param)
            dist = np.linalg.norm(translation)
            if dist > self.max_dist:
                print("dist:", dist)
                return frame, None, None, None

            # convert to degree
            rotation = rotation_rad * 57.2957795131
            if self.show_image:
                # draw box around object
                self.frame = self.draw_box_around_object(self.frame, points_2d)
                # show object position and orientation value to frame
                self.frame = self.draw_values(self.frame, translation, rotation)

        return self.frame, translation, rotation_rad, self.data


    # applying homography matrix as inference of perpective transformation
    def output_3d_points(self, marker_lenght):

        x0 = marker_lenght / 2
        y0 = marker_lenght / 2

        x1 = -marker_lenght / 2
        y1 = -marker_lenght / 2
        # corner_pts_3d = np.float32([[x0, y0, 0], [x0, y1, 0], [x1, y1, 0], [x1, y0, 0]])
        corner_pts_3d = np.float32([[x1, y0, 0], [x0, y0, 0], [x0, y1, 0], [x1, y1, 0]])

        return corner_pts_3d

    # solving pnp using iterative LMA algorithm
    def iterative_solve_pnp(self, object_points, image_points, camera_parameters, camera_distortion_param):
        # image_points = image_points.reshape(-1, 2)
        retval, rotation, translation = cv2.solvePnP(object_points, image_points, camera_parameters,
                                                     camera_distortion_param,
                                                     flags = cv2.SOLVEPNP_IPPE_SQUARE)
        if retval is False:
            return None, None
        return rotation, translation

    # drawing box around object
    def draw_box_around_object(self, frame, dst):
        for i in range(len(dst)):
            frame = cv2.putText(frame, (i+1).__str__(), (int(dst[i][0]), int(dst[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        return cv2.polylines(frame, [np.int32(dst)], True, 255, 3)

    def is_blur(self, image):
        """
        Retund image state
        :param image:
        :param threshold:
        :return:
        """
        val = cv2.Laplacian(image, cv2.CV_64F).var()
        if val < self.threshold:
            return True, val
        else:
            return False, val

    def draw_values(self, frame, translation, rotation):
        # showing object position and orientation value to frame
        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(frame, 'position(cm)', (10, 30), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, 'x:' + str(round(translation[0], 2)), (250, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'y:' + str(round(translation[1], 2)), (350, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'z:' + str(round(translation[2], 2)), (450, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(frame, 'orientation(degree)', (10, 60), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, 'x:' + str(round(rotation[0], 2)), (250, 60), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'y:' + str(round(rotation[1], 2)), (350, 60), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'z:' + str(round(rotation[2], 2)), (450, 60), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'Data: ', (10, 90), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, "'" + str(self.data) + "'", (250, 90), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        return frame