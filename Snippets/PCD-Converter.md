## bin-to-pcd / pcd-to-bin 

> PCL-Python 기반 

```python 
# from Bin 
raw_lidar = np.fromfile('/datasets/testing/velodyne/000001.bin', dtype=np.float32).reshape((-1, 4))
#points = np.fromfile(v_path, dtype=np.float32, count=-1).reshape([-1, num_features]) #SECOND code 
# to Bin 
pc = pcl.load("./sample_lcas.pcd")
pa = pc.to_array()
pa.tofile('sample_lcas.bin')

# to Bin
pc = pcl.load("/workspace/_rosbag/office_bg_2018_10_22_pcd/1540261303.747807979.pcd")
pc_rgb=XYZ_to_XYZRGB(pc,[0,0,0] )
pa = pc_rgb.to_array()

pa.tofile('/workspace/_pcd/1540261303.747807979.bin')
```



---

## txt-to-pcd

> Open3D-Python 기반 

```python 
import os
import subprocess
import shutil
import open3d

from dataset.semantic_dataset import all_file_prefixes


def wc(file_name):
    out = subprocess.Popen(
        ["wc", "-l", file_name], stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    ).communicate()[0]
    return int(out.partition(b" ")[0])


def prepend_line(file_name, line):
    with open(file_name, "r+") as f:
        content = f.read()
        f.seek(0, 0)
        f.write(line.rstrip("\r\n") + "\n" + content)


def point_cloud_txt_to_pcd(raw_dir, file_prefix):
    # File names
    txt_file = os.path.join(raw_dir, file_prefix + ".txt")
    pts_file = os.path.join(raw_dir, file_prefix + ".pts")
    pcd_file = os.path.join(raw_dir, file_prefix + ".pcd")

    # Skip if already done
    if os.path.isfile(pcd_file):
        print("pcd {} exists, skipped".format(pcd_file))
        return

    # .txt to .pts
    # We could just prepend the line count, however, there are some intensity value
    # which are non-integers.
    print("[txt->pts]")
    print("txt: {}".format(txt_file))
    print("pts: {}".format(pts_file))
    with open(txt_file, "r") as txt_f, open(pts_file, "w") as pts_f:
        for line in txt_f:
            # x, y, z, i, r, g, b
            tokens = line.split()
            tokens[3] = str(int(float(tokens[3])))
            line = " ".join(tokens)
            pts_f.write(line + "\n")
    prepend_line(pts_file, str(wc(txt_file)))

    # .pts -> .pcd
    print("[pts->pcd]")
    print("pts: {}".format(pts_file))
    print("pcd: {}".format(pcd_file))
    point_cloud = open3d.read_point_cloud(pts_file)
    open3d.write_point_cloud(pcd_file, point_cloud)
    os.remove(pts_file)


if __name__ == "__main__":
    # By default
    # raw data: "dataset/semantic_raw"
    current_dir = os.path.dirname(os.path.realpath(__file__))
    dataset_dir = os.path.join(current_dir, "dataset")
    raw_dir = os.path.join(dataset_dir, "semantic_raw")

    for file_prefix in all_file_prefixes:
        point_cloud_txt_to_pcd(raw_dir, file_prefix)
```
> PCL-Cpp 기반 :  https://blog.csdn.net/qq_22170875/article/details/90140851




---