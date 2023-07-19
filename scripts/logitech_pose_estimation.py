import pyzed.sl as sl
import cv2
import numpy as np
import quaternionic
import rospy
from std_msgs.msg import String, Bool



class ArucoDetection():
    def __init__(self):
        # rospy.init_node('camera_listener', anonymous=True)
        # self.image_sub = rospy.Subscriber("/aruco_detection_activation", Bool, self.loop)
        # self.obj_pub = rospy.Publisher("/aruco_detection", String, queue_size=10)
        self.trasl_list = []
        self.scene_markers = [100, 10, 0]
        self.smoothing_dict = {}
        self.smoothing_window = 50


    def tf2quat_tr(self, tf):
        quat = quaternionic.array.from_rotation_matrix(tf[:3, :3])
        trasl = tf[:3, 3]
        return trasl, quat
    
    def smooth_traslations(self, traslations, ids):
        ids = ids.flatten()

        for id in ids:
            if id in self.smoothing_dict.keys():
                if len(self.smoothing_dict[id]) > self.smoothing_window:
                    self.smoothing_dict[id] = self.smoothing_dict[id][1:]
                self.smoothing_dict[id].append(traslations[np.where(ids == id)])
            else:
                self.smoothing_dict[id] = [traslations[np.where(ids == id)]]

        smoothed_traslations = np.zeros((len(self.smoothing_dict.keys()), 3, 1))

        for key in self.smoothing_dict.keys():
            if key in ids:
                smoothed_traslations[np.where(ids == key)] = (np.mean(self.smoothing_dict[key], axis=0))
        return np.asarray(smoothed_traslations).reshape(-1, 3,1)

    def get_transform_matrix(self, r_mat, t_vec):
        tf_mat = np.concatenate((r_mat, t_vec.reshape(3,1)), axis=1)
        tf_mat = np.concatenate((tf_mat, np.array([[0, 0, 0, 1]])), axis=0)
        return tf_mat
    
    def moving_average_filter(self, sample, sequence, window_size):
            if len(sequence) > window_size:
                sequence.append(sample)
                sequence=sequence[1:]
            if len(sequence) == window_size:
                yield sum(sequence) / len(sequence)

    def get_center_depth(self, depth_map, corner):
        center = np.mean(corner, axis=0)

        return depth_map[int(center[1]), int(center[0])] / 1000

    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion, depth_map):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''

        
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        i = 0
        for i in range(len(corners)):
            nada, R, t = cv2.solvePnP(marker_points, corners[i], mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            # t[2] = self.get_center_depth(depth_map, corners[i][0])
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return np.asarray(rvecs), np.asarray(tvecs), trash

    def init_zed(self, resolution):
        # Create a ZED camera object
        zed = sl.Camera()

        # Set configuration parameters
        input_type = sl.InputType()

        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = resolution
        init.depth_mode = sl.DEPTH_MODE.QUALITY
        init.coordinate_units = sl.UNIT.MILLIMETER

        # Open the camera
        err = zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS :
            print(repr(err))
            zed.close()
            exit(1)

        # Set runtime parameters after opening the camera
        runtime = sl.RuntimeParameters()
        runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        image_size = zed.get_camera_information().camera_resolution
        print(image_size.width, image_size.height)
        image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
        depth_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.F32_C1)
        return zed, image_size, image_zed, depth_zed


    def grab_zed_frame(self, zed, image_size, depth_zed, image_zed):
        if zed.grab() == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image in sl.Mat
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            # Use get_data() to get the numpy array
            image_ocv = image_zed.get_data()
            
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH, sl.MEM.CPU, image_size)
            depth_ocv = depth_zed.get_data()
            return image_ocv, depth_ocv

    def aruco_display(self, corners, ids, rejected, image):
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                print("[Inference] ArUco marker ID: {}".format(markerID))
                # show the output image
        return image



    def loop(self):

        static_rot = np.asarray([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        # baxter2ref = np.asarray([   [ 1.0,  0.0,  0.0,  0.668],
        #                             [ 0.0, -1.0, -0.0, -0.245],
        #                             [-0.0,  0.0, -1.0, -0.324],
        #                             [ 0.0,          0.0,          0.0,          1.0        ]])
        
        baxter2ref = np.asarray([[0.999, 0.008, 0.033, 0.658],
                                [0.012, -0.994, -0.113, -0.000],
                                [0.032, 0.113, -0.993, -0.280],
                                [0.000, 0.000, 0.000, 1.000]])
        # baxter2ref[2,3] += 0.04


        logi2ref = np.load("/home/index1/index_ws/src/zed_cv2/cam2ref.npy")
        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
        calibration_matrix_path = "/home/index1/index_ws/src/zed_cv2/zed2_calibration_matrix.npy"
        calibration_matrix = np.load(calibration_matrix_path)   

        distortion_path = "/home/index1/index_ws/src/zed_cv2/zed2_distortion.npy"
        distortion = np.load(distortion_path)

        print('\n',distortion)

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) #cv2.aruco.DICT_ARUCO_ORIGINAL
        arucoParams = cv2.aruco.DetectorParameters()
        arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

        # cap = cv2.VideoCapture(0)

        # #resolution stuff
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        # width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        # height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # print(width, height)

        zed, image_size, image_zed, depth_zed = self.init_zed(sl.RESOLUTION.HD2K)

        # print(zed.get_camera_information())
        real_ids = []
        iterations = 0

        while iterations<30:
            # ret, image_ocv = cap.read()
            image_ocv, depth_map = self.grab_zed_frame(zed, image_size, depth_zed, image_zed)
            depth_map_resize = cv2.resize(depth_map, (1280, 720))
            cv2.imshow("depth_map", depth_map_resize)
            # Display the left image from the numpy array
            image_ocv_grey = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = arucoDetector.detectMarkers(image_ocv_grey)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

            refined_corners = []
            for i in range(len(corners)):
                refined_corners.append(cv2.cornerSubPix(image_ocv_grey, corners[i], (3, 3), (-1, -1), criteria))

            # if ids is not None:
            #     for id in ids:
            #         if id not in real_ids:
            #             real_ids.append(id)
    
            # rvec, tvec, _ = self.estimatePoseSingleMarkers(corners, 0.25, calibration_matrix, distortion)
            rvec, tvec, _ = self.estimatePoseSingleMarkers(refined_corners, 0.056, calibration_matrix, distortion, depth_map)
            
            # self.trasl_list.append(tvec)
            # tvec = self.moving_average_filter(tvec, self.trasl_list, 10)

            rod = [cv2.Rodrigues(r)[0] for r in rvec]
            # # print(np.where(ids == 100))
            ref_row = np.where(ids == 100)
            # pos_row = np.where(ids == 5)

            pose_dict = {}

            if ids is not None:
                smoothed_traslations = self.smooth_traslations(tvec, ids)

                for id in ids:
                    if id not in real_ids:
                            real_ids.append(id)
                    if id in self.scene_markers:
                        row_idx = np.where(ids == id)
                        # print(row_idx)

                        if len(row_idx[0]) != 0:
                            row_idx = row_idx[0][0]
                            ref_rot = rod[row_idx]
                            ref_rot = np.dot(ref_rot, static_rot)
                            rvec[row_idx] = cv2.Rodrigues(ref_rot)[0]
                            ref_trasl = smoothed_traslations[row_idx]
                            ref_tf = self.get_transform_matrix(ref_rot, ref_trasl)
                            trasl, r = self.tf2quat_tr(np.dot(baxter2ref, np.dot(np.linalg.inv(logi2ref), ref_tf)))
                            print(f'ID: {id}, trasl: {trasl}, rot: {r.normalized}')
                            pose_dict[id[0]] = [trasl, r.normalized]
                        print('-----------------')
                print(pose_dict)
                iterations += 1
                if iterations == 30-1:
                    if pose_dict:
                        return pose_dict
                    else:
                        iterations = 0

            # if len(ref_row[0]) != 0:
            #     ref_row = ref_row[0][0]


            #     ref_rot = rod[ref_row]
            #     ref_rot = np.dot(ref_rot, static_rot)
            #     rvec[ref_row] = cv2.Rodrigues(ref_rot)[0]
            #     ref_trasl = tvec[ref_row]


            #     ref_tf = self.get_transform_matrix(ref_rot, ref_trasl)
            #     print(ref_tf)
            #     # bax2obj = np.dot(baxter2camera, ref_tf)
            #     # print(bax2obj)
            #     # trasl, r = self.tf2quat_tr(np.dot(np.linalg.inv(logi2ref), ref_tf))
            #     # print(np.dot(np.linalg.inv(logi2ref), ref_tf))
            #     # print(trasl)
            #     # print(r)

            #     trasl, r = self.tf2quat_tr(np.dot(baxter2ref, np.dot(np.linalg.inv(logi2ref), ref_tf)))
            #     # print(np.dot(baxter2ref, np.dot(np.linalg.inv(logi2ref), ref_tf)))
            #     # print(trasl)
            #     # print(r)

            #     # return trasl, r.normalized
            #     # print(np.dot(baxter2ref, np.linalg.inv(logi2ref)))
            #     # print(np.dot(np.linalg.inv(logi2ref), ref_tf))
            #     # np.save('/home/index1/index_ws/src/zed_cv2/logi2ref.npy', ref_tf)
            #     # pos_tf = get_transform_matrix(pos_rot, pos_trasl)
            #     # np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
            #     # ref_to_pos_tf = np.dot(np.linalg.inv(ref_tf), pos_tf)

            #     # print(ref_to_pos_tf)

            # # for c in rod:
            # #     print(c)
            # # # print(rvec)
            # # print(tvec)
            # # print('--------------')
            # quats = []
            # # for i in range(len(rvec)):
            # #     # q = quaternionic.array.from_euler_angles(rvec[i][0], rvec[i][1], rvec[i][2])
            # #     # quats.append(q)
            # #     # print(f'w: {quats[i].w}, x: {quats[i].x}, y: {quats[i].y}, z: {quats[i].z}')
            # #     print(f'w: {rvec[i][0]}, x: {rvec[i][1]}, y: {rvec[i][2]}, z: {rvec[i][3]}')
            # # print(quats)
            # if ids is not None: print(len(ids))
            # image_ocv = self.aruco_display(corners, ids, rejected, image_ocv)
            # # print(rvec.shape)
            # # print(tvec.shape)

            # if tvec.shape[0] > 0 and rvec.shape[0] > 0:
            #     for j in range(tvec.shape[0]):
            #         cv2.drawFrameAxes(image_ocv, calibration_matrix, distortion, rvec[j], tvec[j], 0.1) 
                    
            image_resize = cv2.resize(image_ocv, (1280, 720))
            cv2.imshow("Image", image_resize)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.obj_pub.publish('_'.join([str(id) for id in real_ids]))
        cv2.destroyAllWindows()

    def listener(self):
        rospy.loginfo("I am listening to the camera")
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    HD = ArucoDetection()
    HD.loop()