import numpy as np
import struct 
import open3d as o3d
import os

def convert_kitti_bin_to_pcd(binFilePath, outputPath):
    size_float = 4

    for i in range(len(os.listdir(binFilePath))):
        list_pcd = []
        #bf = binFilePath+"{i}.bin".format(i=str(i).zfill(10))
        bf = binFilePath+"{i}.bin".format(i=str(i).zfill(6))
        nf = outputPath+f"{i}.pcd"
    
        try: 
            with open(bf, "rb") as f: 
                byte = f.read(size_float*4)
                while byte:
                    x, y ,z, intensity = struct.unpack("ffff", byte)
                    list_pcd.append([x, y, z])
                    byte = f.read(size_float * 4)
            np_pcd = np.asarray(list_pcd)
            pcd = o3d.geometry.PointCloud()
            v3d = o3d.utility.Vector3dVector
            pcd.points = v3d(np_pcd)
            o3d.io.write_point_cloud(nf, pcd)
        except FileNotFoundError:
            print(f"file {i} wasn't found")

if __name__ == "__main__":
    #convert_kitti_bin_to_pcd('velodyne_points/data/', 'velodyne_points/data_pcd/')
    convert_kitti_bin_to_pcd('airsim_data/Drone1/', 'airsim_data/Drone1_pcd/')
