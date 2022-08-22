import cv2 as cv
import numpy as np
import glob
import matplotlib.pyplot as plt
import copy
import open3d as o3d
import pathlib

#Performing Camera Calibration
def calibration(matrix_x: int, matrix_y: int, image_dir: str, output_mtx: str, output_dist: str):
    print("Calibrating...")
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((matrix_x*matrix_y,3), np.float32)
    objp[:,:2] = np.mgrid[0:matrix_x,0:matrix_y].T.reshape(-1,2)

    images = glob.glob(image_dir)
    # Arrays to store object points and image points from all the images.
    objpoints = []
    imgpoints = []
    #Cycle through all calibration images and find the object points and image points
    for fname in images:
        print("Processing: ", fname)
        img = cv.imread(fname)
        #Convert to grayscale
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (matrix_x,matrix_y), None)

        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    #Save camera calibration data to file
    print("Saving calibration data to: ", output_mtx, output_dist)
    #Save Camera matrix
    np.savetxt(output_mtx, mtx)
    #Save Distortion coefficients
    np.savetxt(output_dist, dist)  

#Iterate through R and t values with triangulated points
#Used to account for rotation
def adjuster(iteration: int, R_values: list, t_values: list, triangulatedpoints: np.ndarray):
    for i in range(iteration)[::-1]:
        if i == (iteration-1):
            adjustedtriangulated = triangulatedpoints
        adjustedtriangulated = np.matmul(R_values[i], adjustedtriangulated) + t_values[i]
    return adjustedtriangulated

#Undistort images using camera calibration data
def undistort(images: str, output_dir: str, mtx: str, dist: str):
    print("Undistorting...")
    i = -1
    images = glob.glob(images)
    #Load distortion coefficients and camera matrix
    dist = np.loadtxt(dist)
    dist = np.array(dist)
    mtx = np.loadtxt(mtx)
    mtx = np.array(mtx)
    #Cycle through all images and undistort them
    for fname in images:
        print("Processing: ", fname)
        i += 1
        img = cv.imread(fname)
        h, w = img.shape[:2]
        #Find new camera matrix
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        #Undistort image
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        #Crop the image
        dst = dst[y:y+h, x:x+w]
        cv.imwrite(output_dir + str(i) + ".jpg", dst)
    print("Saved undistorted images to: ", output_dir)

#Triangulation from 2 images
def triangulate(img1: str, img2: str, mtx: str, dist: str, algorithm: str, output_txt: str):
    print("Triangulating...")
    #Load the images
    img1 = cv.imread(img1, cv.IMREAD_GRAYSCALE)
    img2 = cv.imread(img2, cv.IMREAD_GRAYSCALE)
    #Using SIFT to find keypoints and descriptors
    if algorithm.upper() == "SIFT":
        #SIFT
        print("Calculating SIFT...")
        sift = cv.SIFT_create()
        #Use previous image to find keypoints and descriptors or run new set
        kp1, des1 = sift.detectAndCompute(img1, None)
        #Find keypoints and descriptors for image 2
        kp2, des2 = sift.detectAndCompute(img2, None)
        #Passing through FLANN matcher to match points
        print("Calculating Flann...")
        FLANN_INDEX_KTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)
    #Using ORB to find keypoints and descriptors
    if algorithm.upper() == "ORB":
        #ORB
        print("Calculating ORB...")
        orb = cv.ORB_create(nfeatures=100000)
        #Use previous image to find keypoints and descriptors or run new set
        kp1, des1 = orb.detectAndCompute(gray, None)
        kp2, des2 = orb.detectAndCompute(img2, None)
        FLANN_INDEX_LSH = 6
        #Using Brute Force matcher to find matches
        print("Calculating with Brute Force...")
        bf = cv.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)
    #Load Camera matrix and distortion coefficients
    mtx = np.loadtxt(mtx)
    dist = np.loadtxt(dist)
    mtx = np.array(mtx)
    dist = np.array(dist)
    pts1 = []
    pts2 = []
    #Using Lowe's ratio test to filter out bad matches
    for i, (m, n) in enumerate(matches):
        if m.distance < 0.8*n.distance:
            pts1.append(kp1[m.queryIdx].pt)
            pts2.append(kp2[m.trainIdx].pt)
    #Convert to intergers
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    #Find essential matrix
    E, mask = cv.findEssentialMat(pts1, pts2, cameraMatrix= mtx, method = cv.RANSAC, prob = 0.999, threshold = 1.0)
    #Delete outliers
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]
    #Find camera rotation and translation
    ret, R, t, mask, triangulate = cv.recoverPose(E, pts1, pts2, cameraMatrix = mtx, distanceThresh = 30, triangulatedPoints = True)
    #Convert homogeneous coordinates 
    triangulate = triangulate / triangulate[None, 3]
    #Save triangulated points to file
    np.savetxt(output_txt, triangulate)
    print("Saved triangulated points to: ", output_txt)

#Triangulation from multiple images
def multitriangulate(image_dir: str, mtx: str, dist: str, algorithm: str, output_txt: str, ply="False"):
    print("Multi Point Triangulating...")
    #Load the images
    images = glob.glob(image_dir)
    images = sorted(images)
    #Load Camera matrix and distortion coefficients
    mtx = np.loadtxt(mtx)
    dist = np.loadtxt(dist)
    mtx = np.array(mtx)
    dist = np.array(dist)
    kp = None
    des = None
    proj = None
    old_R = None
    old_t = None
    i = 0
    R_list = []
    t_list = []
    trinagulatearray = None
    #Cycle through all images and triangulate points
    for image in images:
        i += 1
        gray = cv.imread(image, cv.IMREAD_GRAYSCALE)
        print("Processing: ", image)
        try:
            img2 = cv.imread(images[images.index(image) + 1], cv.IMREAD_GRAYSCALE)
            print("Processing: ", images[images.index(image) + 1])
        except:
            img2 = cv.imread(images[0], cv.IMREAD_GRAYSCALE)
            print("Processing: ", images[0])
        #gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        #Using SIFT to find keypoints and descriptors
        if algorithm.upper() == "SIFT":
            #SIFT
            print("Calculating SIFT...")
            sift = cv.SIFT_create()
            #Use previous image to find keypoints and descriptors or run new set
            if kp is None:
                kp1, des1 = sift.detectAndCompute(gray, None)
            else:
                kp1 = kp
                des1 = des
            #Find keypoints and descriptors for image 2
            kp2, des2 = sift.detectAndCompute(img2, None)
            kp = kp2
            des = des2
            #Passing through FLANN matcher to match points
            print("Calculating Flann...")
            FLANN_INDEX_KTREE = 1
            index_params = dict(algorithm = FLANN_INDEX_KTREE, trees = 5)
            search_params = dict(checks = 50)
            flann = cv.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1, des2, k=2)
        #Using ORB to find keypoints and descriptors
        if algorithm.upper() == "ORB":
            #ORB
            print("Calculating ORB...")
            orb = cv.ORB_create(nfeatures=100000)
            #Use previous image to find keypoints and descriptors or run new set
            if kp is None:
                kp1, des1 = orb.detectAndCompute(gray, None)
            else:
                kp1 = copy.copy(kp)
                des1 = copy.copy(des)
            kp2, des2 = orb.detectAndCompute(img2, None)
            kp = copy.copy(kp2)
            des = copy.copy(des2)
            FLANN_INDEX_LSH = 6
            #Using Brute Force matcher to find matches
            print("Calculating with Brute Force...")
            bf = cv.BFMatcher()
            matches = bf.knnMatch(des1, des2, k=2)
        #SURF
        # print("Calculating SURF...")
        # surf = cv.xfeatures2d.SURF_create(400)
        # if kp is None:
        #     kp1, des1 = surf.detectAndCompute(gray, None)
        # else:
        #     kp1 = kp
        #     des1 = des
        # kp2, des2 = surf.detectAndCompute(img2, None)
        # kp = kp2
        # des = des2
        # FLANN_INDEX_KTREE = 1
        # index_params = dict(algorithm = FLANN_INDEX_KTREE, trees = 5)
        # search_params = dict(checks = 50)
        # flann = cv.FlannBasedMatcher(index_params, search_params)
        # matches = flann.knnMatch(des1, des2, k=2)
        # matches = sorted(matches, key = lambda x:x.distance)
        print(len(matches))
        pts1 = []
        pts2 = []
        #Using Lowe's ratio test to filter out bad matches
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                pts1.append(kp1[m.queryIdx].pt)
                pts2.append(kp2[m.trainIdx].pt)
        #Convert to floats
        pts1 = np.float32(pts1)
        pts2 = np.float32(pts2)       
        print("Calculating Homography...")
        #Finding essential matrix
        print(pts1.shape, pts2.shape)
        E, mask = cv.findEssentialMat(pts1, pts2, cameraMatrix= mtx, method = cv.RANSAC, prob = 0.999, threshold = 1.0)
        #Filter out outliers
        print(E)
        pts1 = pts1[mask.ravel() == 1]
        pts2 = pts2[mask.ravel() == 1]
        #Find camera rotation and translation
        try:
            ret, R, t, mask, triangulate = cv.recoverPose(E, pts1, pts2, cameraMatrix = mtx, distanceThresh = 30, triangulatedPoints = True)
            triangulate = triangulate / triangulate[None, 3]
            triangulate = np.delete(triangulate, 3, 0) 
            #Save R and t to list
            if old_R is not None:
                triangulate = np.matmul(old_R, triangulate) + old_t
            old_R = R
            old_t = t
            #Run iterative triangulation to take account for rotation and translation
            if trinagulatearray is None:
                trinagulatearray = triangulate
            else:
                triangulate = adjuster(len(R_list), R_list, t_list, triangulate)
                trinagulatearray = np.hstack((trinagulatearray, triangulate))
            R_list.append(R)
            t_list.append(t)
            print("Saving Triangulated Points...")
            print(trinagulatearray.shape)
        except:
            print("Not enough points")
            pass
    #Converting to PLY format
    if ply == True:
        file = np.rot90(trinagulatearray)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(file)
        o3d.io.write_point_cloud(output_txt + ".ply", pcd)
    else: 
        np.save(output_txt, trinagulatearray)
    print("Saved triangulated points to: ", output_txt)

def loadPoints(triangulated):
    #Load triangulated points from file
    triangulated = np.load(triangulated)
    fig = plt.figure()
    #Equalizing using https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
    ax = plt.axes(projection='3d')
    X = triangulated[0]
    Y = triangulated[1]
    Z = triangulated[2]
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(X.max()+X.min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(Y.max()+Y.min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(Z.max()+Z.min())
    #Plot points
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')
    graph = ax.scatter3D(X, Y, X, cmap='Greens')
    plt.grid()
    plt.show()

#Poisson Reconstruction
def npytoply(input_file, output_file):
    file = np.load(input_file)
    file = np.rot90(file)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(file)
    o3d.io.write_point_cloud(output_file + ".ply", pcd)


def reconstuction(input_file, output_file, alpha):
    inputsplit = pathlib.Path(input_file).suffix
    print(inputsplit)
    if inputsplit == ".ply":
        pcd = o3d.io.read_point_cloud(input_file)
        pcd.estimate_normals()
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(output_file + ".ply", mesh)
        o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
        #print(pcd)
        # distance = pcd.compute_nearest_neighbor_distance()
        # print(distance)
        # distance = np.mean(distance)
        # distance = distance * 3
        # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([distance, distance*3]))
        # o3d.visualization.draw_geometries([rec_mesh])

    if inputsplit == ".npy":
        file = np.load(input_file)
        file = np.rot90(file)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(file)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(output_file + ".ply", mesh)
        o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    else:
        print("Invalid file type")
