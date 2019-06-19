# Matplot lib.를 이용한 시각화 

> Jupyter 지원 


## 1. 2D 
```python 
import matplotlib.pyplot as plt

def visualization2D_xyz(new_cloud_data):
    # set the size of pyplot charts
    plt.rcParams['figure.figsize'] = (14, 6)

    # split the rgb column into three columns: red, green and blue
    rgb_columns = np.asarray(random_color_gen())
    
    # normalize the rgb values (they should be between [0, 1])
    rgb_columns = (rgb_columns / 255.0).astype(np.float)

    # plot the points of the columns
    plt.scatter(new_cloud_data[:, [0]], -new_cloud_data[:, [1]], color=rgb_columns)

    # scale the axis equally 
    # (Note: append a semicolon to your last line to prevent jupyter notebook 
    # from outputting your last cell's content)
    plt.axis('scaled');
    print("(x) : {:2.1f}m".format(new_cloud_data[:,0:1].max() - new_cloud_data[:,0:1].min()))
    print("(y) : {:2.1f}m".format(new_cloud_data[:,1:2].max() - new_cloud_data[:,1:2].min()))
    print("(z) : {:2.1f}m".format(new_cloud_data[:,2:3].max() - new_cloud_data[:,2:3].min()))

%matplotlib inline
```

## 2. 3D 

```python 
from mpl_toolkits.mplot3d import Axes3D

# Create a figure with a subplot with three axes

def visualization3D_xyz(new_cloud_data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # split the rgb column into three columns: red, green and blue
    rgb_columns = np.asarray(random_color_gen())
    
    # normalize the rgb values (they should be between [0, 1])
    rgb_columns = (rgb_columns / 255.0).astype(np.float)

    ax.scatter(new_cloud_data[:,0], new_cloud_data[:,1], new_cloud_data[:,2], color=rgb_columns);
    print("(x) : {:2.1f}m".format(new_cloud_data[:,0:1].max() - new_cloud_data[:,0:1].min()))
    print("(y) : {:2.1f}m".format(new_cloud_data[:,1:2].max() - new_cloud_data[:,1:2].min()))
    print("(z) : {:2.1f}m".format(new_cloud_data[:,2:3].max() - new_cloud_data[:,2:3].min()))

%matplotlib inline
```

---


```python 
from random import randint
import struct
import ctypes

def random_color_gen():
    """ Generates a random color
    
        Args: None
        
        Returns: 
            list: 3 elements, R, G, and B
    """
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]