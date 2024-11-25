#!/usr/bin/env python3
import cv2 as cv
import numpy as np
from transformations import *
class FisheyeUndist:
    def __init__(self, camera_matrix, dist_coeffs, xi, fov=190, width=1000, height=500, extrinsic=None):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.xi = xi
        self.generatePinhole2(fov, width, height)
        if extrinsic is None:
            self.extrinsic = np.eye(4)
        else:
            self.extrinsic = extrinsic

    def generateUndistMapPinhole(self, R, focal_length, width, height):
        # R: rotation matrix
        # Knew: new camera matrix
        # size: image size
        pts3d = np.zeros((1, width*height, 3), dtype=np.float32)
        c = 0
        for i in range(height):
            for j in range(width):
                p3d = R@[j - width/2, i - height/2, focal_length]
                pts3d[0, c] = p3d
                c += 1
        rvec, tvec = np.array([0., 0., 0.]), np.array([0., 0., 0.])
        pts2d_raw = projectPoints(pts3d, rvec, tvec, self.camera_matrix, self.xi, self.dist_coeffs)
        mapxy = pts2d_raw.reshape((height, width, 2))
        #Map from imgPtsundist to pts2d_raw
        mapx, mapy = cv.convertMaps(mapxy, None, cv.CV_32FC1)
        return mapx, mapy

    def getPinholeCamExtrinsic(self, idx):
        if idx == 0:
            return self.extrinsic @ euler_matrix(0, -np.pi/4, 0, 'sxyz')
        else:
            return self.extrinsic @ euler_matrix(0, np.pi/4, 0, 'sxyz')

    def generatePinhole2(self, fov, width, height):
        pinhole_fov = np.deg2rad(fov - 90)
        focal_gen = width / 2 / np.tan(pinhole_fov / 2)
        R0 = euler_matrix(0, -np.pi/4, 0, 'sxyz')[0:3, 0:3]
        R1 = euler_matrix(0, np.pi/4, 0, 'sxyz')[0:3, 0:3]
        self.focal_gen = focal_gen
        map1 = self.generateUndistMapPinhole(R0, focal_gen, width, height)
        map2 = self.generateUndistMapPinhole(R1, focal_gen, width, height)
        self.maps = [map1, map2]
    
    def undistAll(self, img):
        imgs = []
        for map in self.maps:
            _img = cv.remap(img, map[0], map[1], cv.INTER_AREA)
            imgs.append(_img)
        return imgs
    
    def undist(self, img, idx):
        return cv.remap(img, self.maps[idx][0], self.maps[idx][1], cv.INTER_AREA)

#move this outside

if __name__ == "__main__":
    # Test code
    import argparse
    parser = argparse.ArgumentParser(description='Fisheye undist')
    parser.add_argument("-i","--input", type=str, help="input image file")
    parser.add_argument("-f","--fov", type=float, default=190, help="hoizon fov of fisheye")
    args = parser.parse_args()

    K = np.array([[1162.5434300524314, 0, 660.6393183718625],
            [0, 1161.839362615319,  386.1663300322095],
            [0, 0, 1]])
    D = np.array([-0.17703529535292872, 0.7517933338735744, -0.0008911425891703079, 2.1653595535258756e-05])
    xi = 2.2176903753419963

    undist = FisheyeUndist(K, D, xi, fov=args.fov)
    img = cv.imread(args.input)
    imgs = undist.undistAll(img)
    show = imgs[0]
    for i in range(1, len(imgs)):
        show = cv.hconcat([show, imgs[i]])
    cv.namedWindow("raw", cv.WINDOW_NORMAL|cv.WINDOW_GUI_EXPANDED)
    cv.imshow("raw", img)
    cv.namedWindow("Undist", cv.WINDOW_NORMAL|cv.WINDOW_GUI_EXPANDED)
    cv.imshow("Undist", show)
    cv.waitKey(0)

def projectPoints(pts3d, rvec, t, K, xi, D):
    """
    Projects 3D points onto a 2D image plane using a camera model with radial and tangential distortion.

    Parameters:
    - pts3d: numpy array of shape (n, 3), 3D points in world coordinates.
    - rvec: numpy array of shape (3, 3), rotation vector
    - t: numpy array of shape (3,), translation vector.
    - K: numpy array of shape (3, 3), intrinsic camera matrix.
    - xi: float, parameter for the unit sphere model.
    - D: numpy array of shape (4,), distortion coefficients (k1, k2, p1, p2).

    Returns:
    - pts2d: numpy array of shape (n, 2), projected 2D points in pixel coordinates.
    """
    R = cv.Rodrigues(rvec)[0]
    pts3d = pts3d.reshape(-1, 3)
    n = pts3d.shape[0]
    k1, k2, p1, p2 = D

    # Intrinsic parameters
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    s = K[0, 1]  # Skew coefficient

    # Initialize output array
    pts2d = np.zeros((n, 2), dtype=np.float32)

    for i in range(n):
        # Convert to camera coordinate
        Xw = pts3d[i]
        Xc = np.dot(R, Xw) + t

        # Convert to unit sphere
        norm_Xc = np.linalg.norm(Xc)
        Xs = Xc / norm_Xc

        # Convert to normalized image plane
        xu = np.array([Xs[0] / (Xs[2] + xi), Xs[1] / (Xs[2] + xi)])

        # Add distortion
        r2 = np.dot(xu, xu)
        r4 = r2 * r2

        xd = np.zeros(2)
        xd[0] = xu[0] * (1 + k1 * r2 + k2 * r4) + 2 * p1 * xu[0] * xu[1] + p2 * (r2 + 2 * xu[0] ** 2)
        xd[1] = xu[1] * (1 + k1 * r2 + k2 * r4) + p1 * (r2 + 2 * xu[1] ** 2) + 2 * p2 * xu[0] * xu[1]

        # Convert to pixel coordinates
        final = np.zeros(2)
        final[0] = fx * xd[0] + s * xd[1] + cx
        final[1] = fy * xd[1] + cy

        # print(Xw, final)

        pts2d[i] = final

    return pts2d
