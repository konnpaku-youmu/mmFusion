import os.path
import time
import math
import argparse
import numpy as np
import cv2

import pafy


def system_initialization():
    '''
    Camera calibration
    Frame Alignment
    '''
    # open camera stream
    cam_cap = cv2.VideoCapture()

    # open calibration config
    cfg = cv2.FileStorage("/home/hcrd/Projects/mmFusion/config/system.xml",
                          flags=cv2.FileStorage_READ)
    calib_node = cfg.getNode("Camera").getNode("calib")
    frame_num = int(calib_node.getNode("frameNum").real())
    pattern = calib_node.getNode("pattern").string()
    pattern_size = [calib_node.getNode(
        "width").real(), calib_node.getNode("height").real()]
    spacing = calib_node.getNode("spacing").real()

    data_source = calib_node.getNode("dataSource")
    video_dev = data_source.getNode("uri").string()
    cfg.release()

    # set calibration parameters
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corner_grid = np.zeros((6 * 9, 3), np.float32)
    corner_grid[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    cam_cap.open(video_dev)
    pressed_key = None
    captured = False
    corner_pts = []
    corner_pixels = []
    while not captured:
        ret, frame = cam_cap.read()
        cv2.imshow("Input", frame)
        pressed_key = cv2.waitKey(15)

        if pressed_key == ord('q'):
            break
        elif pressed_key == ord('c'):
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # find patterns
            pattern_found = False
            if pattern == "Chessboard":
                pattern_found, corners = cv2.findChessboardCorners(
                    frame_gray, (9, 6), None)
            elif pattern == "CircleGrid":
                pass

            if pattern_found:
                frame_num -= 1
                corner_pts.append(corner_grid)
                corners_refined = cv2.cornerSubPix(
                    frame_gray, corners, (9, 9), (-1, -1), criteria)
                corner_pixels.append(corners_refined)

                # draw chessboard
                frame_chessboard = cv2.drawChessboardCorners(
                    frame, (9, 6), corners_refined, ret)
                frame_chessboard = cv2.putText(frame_chessboard, "{} images remaining".format(frame_num),
                                               (int(frame.shape[1]*0.8),
                                                int(frame.shape[0]*0.95)),
                                               cv2.FONT_HERSHEY_DUPLEX, 0.6,
                                               (255, 0, 0),
                                               thickness=1, lineType=cv2.LINE_AA)
                cv2.imshow("Calibration Window", frame_chessboard)

                if frame_num == 0:
                    captured = True

    reproj, k, coeffs, rvecs, tvecs = cv2.calibrateCamera(
        corner_pts, corner_pixels, frame_gray.shape[::-1], None, None)

    cv2.destroyAllWindows()
    if(reproj < 30):
        print("Re-projection Error: {}".format(reproj))
        # save parameters
        cfg = cv2.FileStorage("/home/hcrd/Projects/mmFusion/config/cam_calib_result.xml",
                              flags=cv2.FILE_STORAGE_WRITE)
        cfg.write("calibration_timestamp", time.strftime(
            "%Y-%m-%d %H:%M:%S", time.gmtime()))
        cfg.write("device", video_dev)
        cfg.write("cameraMatrix", k)
        cfg.write("distCoeffs", coeffs)
        cfg.write("reprojection_err", reproj)
        cfg.release()
    else:
        print("Re-projection error out of bound...Run initialization again...")

    pressed_key = None
    while pressed_key != ord('q'):
        _, frame = cam_cap.read()
        h, w = frame.shape[:2]
        cfg = cv2.FileStorage("/home/hcrd/Projects/mmFusion/config/cam_calib_result.xml",
                              flags=cv2.FILE_STORAGE_READ)
        camera_mat = cfg.getNode("cameraMatrix").mat()
        camera_coeffs = cfg.getNode("distCoeffs").mat()
        cfg.release()

        dst = cv2.undistort(frame, camera_mat, coeffs, None)
        dst = cv2.putText(dst, "Calibrated. Re-projection error = {0:.2f}".format(reproj),
                          (int(frame.shape[1]*0.65),
                           int(frame.shape[0]*0.95)),
                          cv2.FONT_HERSHEY_DUPLEX, 0.6,
                          (10, 255, 30),
                          thickness=1, lineType=cv2.LINE_AA)
        dst = cv2.hconcat([frame, dst])
        cv2.imshow("Calibrated", dst)

        pressed_key = cv2.waitKey(15)

        if pressed_key == ord('s'):
            print("Image saved")
            cv2.imwrite("./calib_out.png", dst)

    cv2.destroyAllWindows()
    cam_cap.release()


if __name__ == "__main__":
    camera_matrix = None
    distCoeffs = None
    cam = None
    raw_img = None
    undistorted_img = None
    classes = None

    with open("./config/coco.names", 'rt') as f:
        classes = f.read().rstrip('\n').split('\n')

    while not os.path.isfile("./config/cam_calib_result.xml"):
        print("Cannot find camera calibration file...Start calibration process")
        system_initialization()

    # load calibration file
    cfg = cv2.FileStorage("./config/cam_calib_result.xml",
                          cv2.FileStorage_READ)
    cam = cfg.getNode("device").string()
    camera_matrix = cfg.getNode("cameraMatrix").mat()
    distCoeffs = cfg.getNode("distCoeffs").mat()
    cfg.release()

    assert(cam.startswith("rtsp") or cam.startswith("http"))
    assert(camera_matrix.shape == (3, 3))

    # open capture device
    print("Opening {}".format(cam))
    # cam = pafy.new(cam)
    # cam = cam.getbest(preftype="mp4")
    cap = cv2.VideoCapture()
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    cap.open(cam)

    net = cv2.dnn.readNetFromDarknet("./config/yolov3.cfg",
                                     "./config/yolov3.weights")
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    while cv2.waitKey(20) != ord('q'):
        _, raw_img = cap.read()
        new_camera_mat, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distCoeffs,
                                                            raw_img.shape[:2], 0, raw_img.shape[:2])
        undistorted_img = cv2.undistort(raw_img, camera_matrix,
                                        distCoeffs, None, new_camera_mat)
        # undistorted_img = raw_img
        # calculate FOV
        cx, cy, fx, fy = new_camera_mat[0, 2], new_camera_mat[1,
                                                            2], new_camera_mat[0, 0], new_camera_mat[1, 1]
        # h_fov = 2*math.atan(cx / fx) * (180 / math.pi)
        # v_fov = 2*math.atan(cy / fy) * (180 / math.pi)

        # print(h_fov, v_fov)

        h, w = undistorted_img.shape[:2]

        blob = cv2.dnn.blobFromImage(
            undistorted_img, 1/255, (416, 416), [0,0,0], 1, crop=False)

        net.setInput(blob)
        detections = net.forward()
        classIDs = []
        boxes = []
        confs = []

        for res in detections:
            if(res[4] > 0.1):
                scores = res[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                if confidence > 0.4:
                    center_x, center_y = int(
                        res[0] * undistorted_img.shape[1]), int(res[1] * undistorted_img.shape[0])
                    width = int(res[2] * undistorted_img.shape[1])
                    height = int(res[3] * undistorted_img.shape[0])
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    classIDs.append(classID)
                    confs.append(float(confidence))
                    boxes.append([left, top, width, height])

        # indices = cv2.dnn.NMSBoxes(boxes, confs, 0.1, 0.1)

        for i, box in enumerate(boxes):
            typ = classes[classIDs[i]]
            if typ not in ["person", "car", "truck", "bus", "bicycle"]:
                continue
            left, top, width, height = box

            # calculate coord
            h, pitch = 1.988, (-20.77 * math.pi / 180)
            R_cam = np.array([[math.cos(pitch), 0, math.sin(pitch)], [
                             0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])

            pixel_x, pixel_y = int(left + width / 2), int(top + height / 2)
            pixel_yaw, pixel_pitch = 0, 0
            pixel_yaw = math.atan((pixel_x - cx) / fx)
            pixel_pitch = math.atan((cy - pixel_y) / fy)

            pixel_yaw = np.array([[math.cos(pixel_yaw), math.sin(pixel_yaw), 0], [
                -math.sin(pixel_yaw), math.cos(pixel_yaw), 0], [0, 0, 1]])
            pixel_pitch = np.array([[math.cos(pixel_pitch), 0, math.sin(pixel_pitch)], [
                0, 1, 0], [-math.sin(pixel_pitch), 0, math.cos(pixel_pitch)]])

            R_pixel = np.matmul(pixel_yaw, pixel_pitch)

            R_pixel_world = np.matmul(R_cam, R_pixel)

            dir_vec = np.matmul(R_pixel_world, np.array([1, 0, 0]))

            lbd = -0.8 / dir_vec[2]

            dir_vec = lbd * dir_vec

            cv2.rectangle(undistorted_img, (left, top),
                          (left+width, top+height), (255, 0, 255), thickness=2)
            cv2.putText(undistorted_img, "{0}".format(typ), (left, top), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                        (0, 255, 255), thickness=1, lineType=cv2.LINE_AA)
            
            cv2.putText(undistorted_img, "{0}".format(dir_vec), (pixel_x, pixel_y), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                        (0, 0, 255), thickness=1, lineType=cv2.LINE_AA)

        cv2.imshow("mmFusion", undistorted_img)

    cap.release()
