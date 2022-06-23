2020-01-31  
Riccardo Monica <rmonica@ce.unipr.it>  
RIMLab Robotics and Intelligent Machines Laboratory  
Department of Engineering and Architecture  
University of Parma, Italy

Grasping RD Dataset
-------------------

This dataset contains 34 point clouds in PCD format (Point Cloud Library).

Objects of 4 different classes have been scanned using ElasticFusion and an Orbbec Astra-S depth sensor, mounted eye-in-hand on a robot manipulator.

The point clouds have been annotated manually, to provide a ground truth for semantic segmentation algorithms.

- **jugs**: 10 detergent bottles. Parts: *body*, *handle*, *cap*.  
    ![Jugs](jugs.png)
- **hammers**: 7 hammers. Parts: *handle*, *head*.  
    ![Hammers](hammers.png)
- **fourlegs**: 8 quadrupeds (mostly toy horses). Parts: *torso*, *legs*, *head*.  
    ![Fourlegs](fourlegs.png)
- **cups**: 9 cups. Parts: *body*, *handle*.  
    ![Cups](cups.png)