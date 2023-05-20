import copy
import glob

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

indices = ()
rawpoints = []
points2d3darray = []


# Performing Camera Calibration
def calibration(
        matrix_x: int, matrix_y: int, image_dir: str, output_mtx: str, output_dist: str
):
    print("Calibrating...")
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((matrix_x * matrix_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:matrix_x, 0:matrix_y].T.reshape(-1, 2)

    images = glob.glob(image_dir)
    # Arrays to store object points and image points from all the images.
    objpoints = []
    imgpoints = []
    # Cycle through all calibration images and find the object points and image points
    for fname in images:
        print("Processing: ", fname)
        img = cv.imread(fname)
        # Convert to grayscale
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (matrix_x, matrix_y), None)

        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
    # Save camera calibration data to file
    print("Saving calibration data to: ", output_mtx, output_dist)
    # Save Camera matrix
    np.savetxt(output_mtx, mtx)
    # Save Distortion coefficients
    np.savetxt(output_dist, dist)


# Iterate through R and t values with triangulated points
# Used to account for rotation
def adjuster(
        iteration: int, R_values: list, t_values: list, triangulatedpoints: np.ndarray
):
    for i in range(iteration)[::-1]:
        if i == (iteration - 1):
            adjustedtriangulated = triangulatedpoints
        adjustedtriangulated = (
                np.matmul(R_values[i], adjustedtriangulated) + t_values[i]
        )
    return adjustedtriangulated


# Undistort images using camera calibration data
def undistort(images: str, output_dir: str, mtx: str, dist: str):
    print("Undistorting...")
    i = -1
    images = glob.glob(images)
    # Load distortion coefficients and camera matrix
    dist = np.loadtxt(dist)
    dist = np.array(dist)
    mtx = np.loadtxt(mtx)
    mtx = np.array(mtx)
    # Cycle through all images and undistort them
    for fname in images:
        print("Processing: ", fname)
        i += 1
        img = cv.imread(fname)
        h, w = img.shape[:2]
        # Find new camera matrix
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # Undistort image
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        # Crop the image
        dst = dst[y: y + h, x: x + w]
        cv.imwrite(output_dir + str(i) + ".jpg", dst)
    print("Saved undistorted images to: ", output_dir)


# Triangulation from 2 images
def triangulate(
        img1: str, img2: str, mtx: str, dist: str, algorithm: str, output_txt: str
):
    print("Triangulating...")
    # Load the images
    img1 = cv.imread(img1, cv.IMREAD_GRAYSCALE)
    img2 = cv.imread(img2, cv.IMREAD_GRAYSCALE)
    # Using SIFT to find keypoints and descriptors
    if algorithm.upper() == "SIFT":
        # SIFT
        print("Calculating SIFT...")
        sift = cv.SIFT_create()
        # Use previous image to find keypoints and descriptors or run new set
        kp1, des1 = sift.detectAndCompute(img1, None)
        # Find keypoints and descriptors for image 2
        kp2, des2 = sift.detectAndCompute(img2, None)
        # Passing through FLANN matcher to match points
        print("Calculating Flann...")
        FLANN_INDEX_KTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)
    # Using ORB to find keypoints and descriptors
    if algorithm.upper() == "ORB":
        # ORB
        print("Calculating ORB...")
        orb = cv.ORB_create(nfeatures=100000)
        # Use previous image to find keypoints and descriptors or run new set
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        FLANN_INDEX_LSH = 6
        # Using Brute Force matcher to find matches
        print("Calculating with Brute Force...")
        bf = cv.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)
    # Load Camera matrix and distortion coefficients
    mtx = np.loadtxt(mtx)
    dist = np.loadtxt(dist)
    mtx = np.array(mtx)
    dist = np.array(dist)
    pts1 = []
    pts2 = []
    # Using Lowe's ratio test to filter out bad matches
    for i, (m, n) in enumerate(matches):
        if m.distance < 0.8 * n.distance:
            pts1.append(kp1[m.queryIdx].pt)
            pts2.append(kp2[m.trainIdx].pt)
    # Convert to intergers
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    # Find essential matrix
    E, mask = cv.findEssentialMat(
        pts1, pts2, cameraMatrix=mtx, method=cv.RANSAC, prob=0.999, threshold=1.0
    )
    # Delete outliers
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]
    # Find camera rotation and translation
    ret, R, t, mask, triangulate = cv.recoverPose(
        E, pts1, pts2, cameraMatrix=mtx, distanceThresh=30, triangulatedPoints=True
    )
    # Convert homogeneous coordinates
    triangulate = triangulate / triangulate[None, 3]
    # Save triangulated points to file
    np.savetxt(output_txt, triangulate)
    print("Saved triangulated points to: ", output_txt)


# Triangulation from multiple images
def multitriangulate(
        image_dir: str, mtx: str, dist: str, kp_alg: str, sur_alg: str, output: str
):
    print("Performing Intial Triangulate using: ", kp_alg)
    # Load the images
    images = glob.glob(image_dir)
    images = sorted(images)
    # Load Camera matrix and distortion coefficients
    mtx = np.loadtxt(mtx)
    dist = np.loadtxt(dist)
    mtx = np.array(mtx)
    dist = np.array(dist)
    kp = None
    des = None
    old_R = None
    old_t = None
    R_list = []
    t_list = []
    camera_params = []
    points3d = []
    camera_indices = []
    point_indices = []
    points2dout = []
    indexarr = []
    trinagulatearray = None
    if sur_alg.lower() != "poisson" or sur_alg.lower() != "alpha":
        raise Exception("Invalid Surface Reconstruction Algorithm")
    if kp_alg.upper() != "SIFT" or kp_alg.upper() != "ORB" or kp_alg.upper() != "BRISK":
        raise Exception("Invalid Keypoint Algorithm")
    # Cycle through all images and triangulate points
    for image_id, image in enumerate(images):
        points2d = []
        gray = cv.imread(image, cv.IMREAD_GRAYSCALE)
        try:
            img2 = cv.imread(images[image_id + 1], cv.IMREAD_GRAYSCALE)
        except:
            img2 = cv.imread(images[0], cv.IMREAD_GRAYSCALE)
        # Using SIFT to find keypoints and descriptors
        if kp_alg.upper() == "SIFT":
            # SIFT
            sift = cv.SIFT_create()
            # Use previous image to find keypoints and descriptors or run new set
            if kp is None:
                kp1, des1 = sift.detectAndCompute(gray, None)
            else:
                kp1 = kp
                des1 = des
            # Find keypoints and descriptors for image 2
            kp2, des2 = sift.detectAndCompute(img2, None)
            kp = kp2
            des = des2
            # Passing through FLANN matcher to match points
            FLANN_INDEX_KTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KTREE, trees=5)
            search_params = dict(checks=50)
            flann = cv.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1, des2, k=2)
        # Using ORB to find keypoints and descriptors
        if kp_alg.upper() == "ORB":
            # ORB
            orb = cv.ORB_create(
                edgeThreshold=10,
                patchSize=35,
                nlevels=12,
                fastThreshold=20,
                scaleFactor=1.5,
                nfeatures=1000000,
                scoreType=cv.ORB_FAST_SCORE,
            )
            # Use previous image to find keypoints and descriptors or run new set
            if kp is None:
                kp1, des1 = orb.detectAndCompute(gray, None)
            else:
                kp1 = copy.copy(kp)
                des1 = copy.copy(des)
            kp2, des2 = orb.detectAndCompute(img2, None)
            kp = copy.copy(kp2)
            des = copy.copy(des2)
            # Using Brute Force matcher to find matches
            bf = cv.BFMatcher()
            matches = bf.knnMatch(des1, des2, k=2)
        # Using BRISK to find keypoints and descriptors
        if kp_alg.upper() == "BRISK":
            # BRISK
            # print("Calculating BRISK...")
            brisk = cv.BRISK_create()
            # Use previous image to find keypoints and descriptors or run new set
            if kp is None:
                kp1, des1 = brisk.detectAndCompute(gray, None)
            else:
                kp1 = kp
                des1 = des
            kp2, des2 = brisk.detectAndCompute(img2, None)
            kp = kp2
            des = des2
            # Passing through FLANN matcher to match points
            # print("Calculating Flann...")
            FLANN_INDEX_KTREE = 6
            index_params = dict(
                algorithm=FLANN_INDEX_KTREE,
                table_number=6,
                key_size=12,
                multi_probe_level=1,
            )
            search_params = dict(checks=50)
            flann = cv.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1, des2, k=2)
        pts1 = []
        pts2 = []
        # Using Lowe's ratio test to filter out bad matches
        matchcount = 0
        try:
            for i in range(len(matches)):
                if len(matches[i]) != 2:
                    # print("Could not find match for" + str(i))
                    continue
                if matches[i][0].distance < 0.7 * matches[i][1].distance:
                    matchcount += 1
                    pts1.append(kp1[matches[i][0].queryIdx].pt)
                    pts2.append(kp2[matches[i][0].trainIdx].pt)
                    indexarr = pointindeces(
                        kp1[matches[i][0].queryIdx],
                        kp2[matches[i][0].trainIdx],
                        image_id,
                        image_id + 1,
                        matches[i][0].queryIdx,
                        matches[i][0].trainIdx,
                        indexarr,
                    )
                    points2d.append(kp1[matches[i][0].queryIdx].pt)
            pts1 = np.float32(pts1)
            pts2 = np.float32(pts2)
            # Finding essential matrix
            E, mask = cv.findEssentialMat(
                pts1,
                pts2,
                cameraMatrix=mtx,
                method=cv.RANSAC,
                prob=0.999,
                threshold=1.0,
            )
            # Filter out outliers
            pts1 = pts1[mask.ravel() == 1]
            pts2 = pts2[mask.ravel() == 1]
            # Find camera rotation and translation
            ret, R, t, mask, triangulate = cv.recoverPose(
                E,
                pts1,
                pts2,
                cameraMatrix=mtx,
                distanceThresh=30,
                triangulatedPoints=True,
            )
            triangulate = triangulate / triangulate[None, 3]
            triangulate = np.delete(triangulate, 3, 0)
            # Save R and t to list
            if old_R is not None:
                triangulate = np.matmul(old_R, triangulate) + old_t
            old_R = R
            old_t = t
            # Run iterative triangulation to take account for rotation and translation
            if trinagulatearray is None:
                trinagulatearray = triangulate
            else:
                triangulate = adjuster(len(R_list), R_list, t_list, triangulate)
                trinagulatearray = np.hstack((trinagulatearray, triangulate))
            R_list.append(R)
            t_list.append(t)
            t = t.tolist()
            triangulate = np.rot90(triangulate)
            rangle = cv.Rodrigues(R)
            rangle = rangle[0].tolist()
            mtxunrwrap = mtx.ravel()
            camera_params.append(
                [
                    rangle[0][0],
                    rangle[1][0],
                    rangle[2][0],
                    t[0][0],
                    t[1][0],
                    t[2][0],
                    mtxunrwrap[0],
                    mtxunrwrap[2],
                    mtxunrwrap[4],
                    mtxunrwrap[5],
                    dist[0],
                    dist[1],
                    dist[2],
                    dist[3],
                    dist[4],
                ]
            )
            for i in range(len(triangulate)):
                camera_indices.append(image_id)
                point_indices.append(indexarr[i])
                points3d.append(triangulate[i])
                points2dout.append(points2d[i])
        except:
            # If not enough points are found, skip to next image
            print(
                "Not enough points found for image "
                + str(image_id)
                + " and "
                + str(image_id + 1)
            )
            pass
    print("Amount of points:", len(points3d))
    try:
        bundleadjust(
            np.array(camera_params),
            np.array(points3d),
            np.array(camera_indices),
            np.array(point_indices),
            np.array(points2dout),
            output,
            sur_alg,
        )
    except:
        reconstuction(points3d, sur_alg, output)
    print("Done!")


def adjuster(
        iteration: int, R_values: list, t_values: list, triangulatedpoints: np.ndarray
):
    for i in range(iteration)[::-1]:
        if i == (iteration - 1):
            adjustedtriangulated = triangulatedpoints
        adjustedtriangulated = (
                np.matmul(R_values[i], adjustedtriangulated) + t_values[i]
        )
    return adjustedtriangulated


def pointindeces(kp1, kp2, imageidx1, imageidx2, keypt1, keypt2, indexarray):
    """Creates index of the keypoints in the image
    kp1 = openCV keypoints of first image (vector)
    kp2 = openCV keypoints of second image (vector)
    Each element is the index of the keypoints in the images

    Image 1 -> kp1
    Image 2 -> kp2
    matches -> kp1[i] = kp2[j]

    kp1[i] and kp2[j] puts it into rawpoints because no points that matched yet

    Image 2 -> kp1
    Image 3 -> kp2
    matches -> kp1[j] = kp2[k]

    rawpoints[r] = [kp1[i], kp2[j], kp2[k]]

    """
    if len(rawpoints) >= 1:
        """Check if points have matched with existing points in the array, if new points are found, add them to the array, else add the points to the existing array
        ((image#, keypointindx) , kp.index)
        """
        existingmatch = False
        for i in range(len(rawpoints)):
            if kp1 == rawpoints[i][-1][-1]:
                rawpoints[i].append(((imageidx2, keypt2), kp2))
                existingmatch = True
                indexarray.append(i)
        if not existingmatch:
            rawpoints.append([((imageidx1, keypt1), kp1), ((imageidx2, keypt2), kp2)])
            indexarray.append(len(rawpoints) - 1)
    else:
        rawpoints.append([((imageidx1, keypt1), kp1), ((imageidx2, keypt2), kp2)])
        indexarray.append(0)
    return indexarray


def loadPoints(triangulated):
    # Load triangulated points from file
    triangulated = np.load(triangulated)
    fig = plt.figure()
    # Equalizing using https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
    ax = plt.axes(projection="3d")
    X = triangulated[0]
    Y = triangulated[1]
    Z = triangulated[2]
    max_range = np.array(
        [X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]
    ).max()
    Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (
            X.max() + X.min()
    )
    Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (
            Y.max() + Y.min()
    )
    Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (
            Z.max() + Z.min()
    )
    # Plot points
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], "w")
    graph = ax.scatter3D(X, Y, X, cmap="Greens")
    plt.grid()
    plt.show()


## Bundle Adjustment
def bundleadjust(
        camera_params, p3d, camera_indices, point_indices, points_2d, output, sur_alg
):
    dict_pntindx, duplicate_poin_indx = create_duplicate_dict(point_indices)
    optimize(
        camera_params,
        p3d,
        camera_indices,
        point_indices,
        points_2d,
        dict_pntindx,
        duplicate_poin_indx,
        output,
        sur_alg,
    )


def parseparams(cparams):
    cameramatrix = np.zeros((3, 3))
    rvecs = cparams[0:3]
    tvecs = cparams[3:6]
    cameramatrix[0, 0] = cparams[6]
    cameramatrix[0, 2] = cparams[7]
    cameramatrix[1, 1] = cparams[8]
    cameramatrix[1, 2] = cparams[9]
    cameramatrix[2, 2] = 1
    dist = cparams[10:]
    return cameramatrix, rvecs, tvecs, dist


def transfromRt(oldr, newr, oldt, newt):
    R = np.matmul(newr, oldr)
    t = np.matmul(oldr, newt) + oldt
    return R, t


def project(p3d, params, index):
    cameramatrix, R, T, dist = parseparams(params[index])
    R = cv.Rodrigues(R)[0]
    # if index == 0:
    #     proj, _ = cv.projectPoints(points_3d, np.identity(3), np.zeros((3, 1)), cameramatrix, dist)
    for i in range(index + 1):
        try:
            _, newrvecs, newtvecs, _ = parseparams(params[i + 1])
        except:
            _, newrvecs, newtvecs, _ = parseparams(params[0])
        newrvecs = cv.Rodrigues(newrvecs)[0]
        R, T = transfromRt(R, newrvecs, T, newtvecs)
    point_proj, _ = cv.projectPoints(p3d, R, T, cameramatrix, dist)
    return point_proj


def organizer(cindx):
    out = {}
    for i in range(len(cindx)):
        try:
            out[cindx[i]].append(i)
        except:
            out[cindx[i]] = [i]
    return out


def batchproject(p3d, params, camidx, n_cameras):
    organized = organizer(camidx)
    proj = np.array([])
    for j in np.unique(camidx):
        pntinput = np.array([])
        try:
            for i in organized[j]:
                pntinput = np.append(pntinput, p3d[i])
            pntinput = pntinput.reshape((-1, 3))
            proj = np.append(proj, project(pntinput, params, j))
        except:
            pass
    return proj.reshape((-1, 2))


def funresiduals(
        params, n_cameras, n_points, camidx, points_2d, pnt_dict, camera_params
):
    cam_params = np.zeros((n_cameras, 15))
    for i in range(n_cameras):
        cam_params[i][:6] = params[i * 6: i * 6 + 6]
        cam_params[i][6:] = camera_params[i][6:]
    p3d = params[n_cameras * 6:].reshape((n_points, 3))
    points_proj = batchproject(p3d, cam_params, camidx, n_cameras)
    err = np.sqrt(np.square(points_proj - points_2d)).ravel()
    ## Total error reporjection + 3D point error
    ## alpha beta terms
    p3d_err = 0
    for key, value in pnt_dict.items():
        if len(value) > 1:
            for i in range(len(value) - 1):
                # p3d_err += np.linalg.norm(p3d[value[i]] - p3d[value[i + 1]])
                err = np.append(
                    err, np.sqrt(np.square(p3d[value[i]] - p3d[value[i + 1]])) * 1000
                )
    # err += p3d_err * 1000
    return err


def create_duplicate_dict(pnt_indices):
    out = dict()
    total_length = 0
    for i in range(len(pnt_indices)):
        if pnt_indices[i] in out:
            out[pnt_indices[i]].append(i)
            total_length += 1
        else:
            out[pnt_indices[i]] = [i]
    return out, total_length


def bundleadjustment_sparsity(
        n_cameras, n_points, cindx, point_indices, duplicate_poin_indx
):
    m = cindx.size * 2 + duplicate_poin_indx * 3
    n = n_cameras * 6 + n_points * 3
    A = lil_matrix((m, n), dtype=int)
    i = np.arange(cindx.size)
    for s in range(6):
        A[2 * i, cindx * 6 + s] = 1
        A[2 * i + 1, cindx * 6 + s] = 1
    for s in range(3):
        A[2 * i, n_cameras * 6 + point_indices * 3 + s] = 1
        A[2 * i + 1, n_cameras * 6 + point_indices * 3 + s] = 1
    return A


def optimize(
        camera_params,
        p3d,
        cindx,
        point_indices,
        points_2d,
        pnt_indx_dict,
        duplicate_poin_indx,
        output,
        sur_alg,
):
    n_cameras = len(np.unique(cindx))
    n_points = p3d.shape[0]
    n = 6 * n_cameras + 3 * n_points
    m = 2 * points_2d.shape[0] + duplicate_poin_indx * 3
    params = np.hstack((camera_params[:, 0:6].ravel(), p3d.ravel()))
    f0 = funresiduals(
        params, n_cameras, n_points, cindx, points_2d, pnt_indx_dict, camera_params
    )
    print("Initial RMS:", np.sqrt(np.dot(f0, f0) / m))
    A = bundleadjustment_sparsity(
        n_cameras, n_points, cindx, point_indices, duplicate_poin_indx
    )
    print("Optimizing")
    res = least_squares(
        funresiduals,
        params,
        verbose=2,
        x_scale="jac",
        ftol=1e-8,
        jac_sparsity=A,
        xtol=1e-8,
        method="trf",
        args=(n_cameras, n_points, cindx, points_2d, pnt_indx_dict, camera_params),
        max_nfev=200,
    )
    print("Final RMS:", np.sqrt(np.dot(res.fun, res.fun) / m))
    newcamera_params, newpts3d = extractCamAndPnts(
        res.x, n_cameras, n_points, camera_params
    )
    try:
        reconstuction(newpts3d, sur_alg, output)
    except:
        print("Error in mesh creation")


def extractCamAndPnts(result, n_cameras, n_points, camera_params):
    newpts3d = result[n_cameras * 6:].reshape((n_points, 3))
    newcamera_params = np.zeros((n_cameras, 15))
    for i in range(n_cameras):
        newcamera_params[i][:6] = result[i * 6: i * 6 + 6]
        newcamera_params[i][6:] = camera_params[i][6:]
    return np.asarray(newcamera_params.reshape((n_cameras, 15))), np.asarray(
        newpts3d.reshape((n_points, 3))
    )


# Poisson Reconstruction
def npytoply(input_file, output_file):
    file = np.load(input_file)
    file = np.rot90(file)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(file)
    o3d.io.write_point_cloud(output_file + ".ply", pcd)


def reconstuction(points, sur_alg, output):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    cel, inlier = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=0.1)
    if sur_alg.lower() == "poisson":
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            cel, depth=8
        )
        o3d.io.write_triangle_mesh(output, mesh)
    elif sur_alg.lower() == "alpha":
        cel.estimate_normals()
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cel, 0.6)
        o3d.io.write_triangle_mesh(output, mesh)
    else:
        print("Invalid surface reconstruction algorithm")
