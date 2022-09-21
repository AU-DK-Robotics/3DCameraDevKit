# PMDCamera (Pico Monstar)

[**Pico Monstar**](https://pmdtec.com/picofamily/monstar/) is a Time-of-Flight (ToF) camera, which can capture 3D data and up to 60fps, and is easy to deploy for your applications as well. 

It should be noted that this repository is not an official guideline but a doable and practice for use of a PMD camera, specifically, the **Pico Monstar**. Regarding to technical specification, please click [here](./specifications) or official website to know more details.

<p align="center">
  <img width="460" height="300" src="./images/pico_monstar.png">
</p>

## Requirements for development

- `PCL 1.8 to 1.12.1 (for visualization) with VS2019/VS2017`

- Royale SDK (provided by [pico_monstar](https://pmdtec.com/picofamily/monstar/), if you can not download, please try to find in "./Specification/*" folder)

## Running (on windows)

### New Version
Only Tested on Windows 10

Make sure "camera_parameters.txt" is placed in the same directory.

``` bash
#	0: indoor room reconstruction, MODE_9_5FPS_1900
#	1: Room scanning indoor navigation, MODE_9_10FPS_900
# 2: 3D object reconstruction, MODE_9_15FPS_600
#	3: Medium size object recognition or face reconstruction, MODE_9_25FPS_300
#	4: Remote collaboration, MODE_5_35PFS_500
#	5: Small object, MODe_5_45FPS_400
#	6: Hand tracking, MODE_5_60FPS
#	7: Mixed Mode
#	8: Mixed Mode
#	9: low noise extended
#	10: Fast acquisition
# 11: very fast acquisition
operate_mode 1
min_x -0.6
max_x 0.6
min_y 0.0
max_y 0.4
min_z 0.1
max_z 1.1
saved_directory_name today_saved
#saved_format only has bin/txt/auto
saved_format bin
#ms
interval_time 100
```


### Older Version
``` bash

# please run following these parameters as below:
PMD_camera.exe min_x max_x min_y max_y min_z max_z saved_directory_name saved_format[bin/txt/auto] interval_time(ms, only for auto)

# For example:
# filter with "-0.6 0.6 0.0 0.4 0.1 1.1", and all data will be saved with .bin, a format that is space saved. 100 is ignored
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved bin 100

# filter with "-0.6 0.6 0.0 0.4 0.1 1.1", and all data will be saved with .txt, a format that can be readable by Notepad. 100 is ignored
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved_2 txt 100

# filter with "-0.6 0.6 0.0 0.4 0.1 1.1", and all data will be saved with .txt and .bin per 100 milesecond.
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved_2 auto 100

# here is the output:
detected 1 camera
Select the MODE_5_60FPS_300mode
write to 2020-08-07-18-34-26.bin(72368)
```

<p align="center">
  <img width="460" height="300" src="./images/sample.png">
</p>
