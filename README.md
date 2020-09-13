# PMDCamera

Retrieve point cloud or depth image using PMD camera([pico_monstar](https://pmdtec.com/picofamily/monstar/))

![PMD_Camera](./images/pico_monstar.png)

## Requirements

- PCL 1.8.1+
- Royale SDK (provided by [pico_monstar](https://pmdtec.com/picofamily/monstar/))

## Obtain & Show point cloud using PMD camera

``` shell
PMD_camera.exe min_x max_x min_y max_y min_z max_z saved_directory_name saved_format[bin/txt]

for example:
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved bin
PMD_Camera.exe -0.6 0.6 0.0 0.4 0.1 1.1 today_saved_2 txt
```

![image](./images/sample.png)

When the key `S` is pressed, a current point cloud will be named after the date and time, and then saved in `./`.

``` shell
detected 1 camera
Select the MODE_5_60FPS_300mode
write to 2020-08-07-18-34-26.bin(72368)
```
