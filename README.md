# Material Estimation from Point Cloud
This library can calculate point cloud material, both when the point intensity is present and when it isn't.

## Library used
Point Cloud Library https://pointclouds.org/

At the time this repository was made, the version of the PCL library used was 1.10.0. 
It was installed with command: 
```
$ sudo apt install libpcl-dev
```
Vector3 calculation library by Jimmy van den Berg https://github.com/jimmyberg/Vector3

##  How to calculate point material
PCPointMaterial and its associated classes calculates material for each single point cloud points. 
This is usually done for every point of the point cloud. The Clustering or averaging of the material can then be done
later manually. 

There are three separate ways you can calculate point material. Depending on which builder is used. 
### Data preparation
Depending on which method below is chosen, you need different data. Here it lists all of them from one of the code snippets as example.
Normally those values would be acquired from the point cloud.
```
float raw_intensity;
Vector3 RGB(r, g, b);
Vector3 point_coordinate(x, y, z);
Vector3 point_normal = PCNormal::getNormalVectorByIndex(cloud_normals, current_index);
Vector3 scanner_position(0, 0, 0);
```

### 1. Point Material with intensity
This is for when intensity information is available. Intensity correction is done by default.
```
PCPointMaterialBuilder* builder = new PCPointMaterialBuilder(raw_intensity, RGB, point_coordinate, point_normal, scanner_position);
PCPointMaterialDirector director;
director.setBuilder(builder);
director.buildPCPointMaterial();
pcmattex::PCPointMaterial point_material = builder->getPCPointMaterial();
```
### 2. Point Material with mock intensity, still with intensity correction
This is for when intensity information isn't available and grayscale is used to mock intensity. Intensity correction is still performed here. 
```
PCPointMaterialBuilder* builder = new MockIntensityPCPointMaterialBuilder(RGB, point_coordinate, scanner_position, point_normal);
PCPointMaterialDirector director;
director.setBuilder(builder);
director.buildPCPointMaterial();
pcmattex::PCPointMaterial point_material = builder->getPCPointMaterial();
```

### 3. Point Material with mock intensity, without intensity correction
This discards intensity correction altogether. 
```
PCPointMaterialBuilder* builder = new MockIntensityNoCorrectionPCPointMaterialBuilder(RGB, point_coordinate);
PCPointMaterialDirector director;
director.setBuilder(builder);
director.buildPCPointMaterial();
pcmattex::PCPointMaterial point_material = builder->getPCPointMaterial();
```