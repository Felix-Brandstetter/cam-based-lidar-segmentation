
#params
path_to_pointclouds = "./data/Extracted-Data/point_clouds"
path_to_images = "./data/Extracted-Data/debayer"
outputdir = "./data/Extrinsic Calibration"
number_of_points = 10 #Number of points which must be selected to start the extrinsic calibration.
#Möglichkeit punkte anzuhängen

import open3d
import numpy as np
import cv2
import multiprocessing
import os
import glob


def extract_picked_points_3D(path_to_pointcloud,queue):
    print("1")
    points = np.load(path_to_pointcloud)
    pointcloud = open3d.geometry.PointCloud()
    pointcloud.points = open3d.utility.Vector3dVector(points)
    vis = open3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pointcloud)
    vis.run()
    vis.destroy_window()
    point_index = vis.get_picked_points()
    picked_3D_points = np.asarray(pointcloud.points)[point_index]
    return_dic = queue.get()
    return_dic["picked_points_3D"] = np.array(picked_3D_points)
    queue.put(return_dic)


def callback_mouse(event, x, y, flags, params):
    print("2")
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(params[0], (x, y), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.imshow("image", params[0])
        params[1].append([x, y])
        print("Picket point: [%s,%s] in 2D Image" %(x,y))


def extract_picked_points_2D(path_to_image,queue):
    image = cv2.imread(path_to_image)
    cv2.namedWindow("image")
    params = [image, []]
    cv2.setMouseCallback("image", callback_mouse, params)
    while True:
        cv2.imshow("image", image)
        if cv2.waitKey(33) == 27:
            cv2.destroyAllWindows()
            break

    return_dic = queue.get()
    return_dic["picked_points_2D"] = np.array(params[1])
    queue.put(return_dic)

if __name__ == "__main__":

    #Point Picking
    picked_points_2D =  []
    picked_points_3D =  []
    pointclouds = iter(sorted(glob.glob(os.path.join(path_to_pointclouds,"*.npy"))))
    images = iter(sorted(glob.glob(os.path.join(path_to_images,"*.png"))))
    while True:
        return_dic = {"picked_points_2D":None, "picked_points_3D":None, }
        queue = multiprocessing.Queue()
        queue.put(return_dic)
        image = next(images)
        pointcloud = next(pointclouds)
        process1 = multiprocessing.Process(target=extract_picked_points_2D, args=(image,queue))
        process2 = multiprocessing.Process(target=extract_picked_points_3D, args=(pointcloud,queue))
        process1.start()
        process2.start()
        process1.join()
        process2.join()
        
        return_dic = queue.get()
        assert(len(return_dic["picked_points_2D"]) == len(return_dic["picked_points_3D"]))

        if len(picked_points_2D) == 0:
            picked_points_2D = return_dic["picked_points_2D"]
            picked_points_3D = return_dic["picked_points_3D"]
        else:  
            picked_points_2D = np.concatenate([picked_points_2D,return_dic["picked_points_2D"]])
            picked_points_3D = np.concatenate([picked_points_3D,return_dic["picked_points_3D"]])
        print("So far you have selected %s out of %s points" %(len(picked_points_2D),number_of_points))
        if len(picked_points_2D) >= number_of_points:
            print("break")
            break
        print("new")

    picked_points_2D =np.append(picked_points_2D,np.ones(shape=(len(picked_points_2D),1)),axis=1)
    picked_points_3D = np.append(picked_points_3D,np.ones(shape=(len(picked_points_3D),1)),axis=1)
    print("Selected 2D points: \n %s" %picked_points_2D)
    print("Selected 3D_points: \n %s" %picked_points_3D)

    np.save(os.path.join(outputdir,"picked_points/picked_3D_points.npy"), picked_points_3D)
    print("saved 3D points")
    np.save(os.path.join(outputdir,"picked_points/picked_2D_points.npy"), picked_points_2D)
    print("saved 2D points")
    
