# PMDCamera

[This](https://pmdtec.com/picofamily/monstar/) is a Time-of-Flight (ToF) camera, which can capture 3D data and up to 60fps. It is easy to deploy for your applications. 

It should be noted that this repository is not an official guideline but a doable and practice for using a PMD camera. According to technical specification, please click [here](./specifications) or official website.

<p align="center">
  <img width="460" height="300" src="./images/pico_monstar.png">
</p>

## Requirements

- PCL 1.8.1+ (for visualization)
- Royale SDK (provided by [pico_monstar](https://pmdtec.com/picofamily/monstar/))

## Running [on windows]

``` python

# please run following these parameters as below:
PMD_camera.exe min_x max_x min_y max_y min_z max_z saved_directory_name saved_format[bin/txt]

# for example:
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved bin
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved_2 txt

# here is the output:
detected 1 camera
Select the MODE_5_60FPS_300mode
write to 2020-08-07-18-34-26.bin(72368)
```

<p align="center">
  <img width="460" height="300" src="./images/sample.png">
</p>
