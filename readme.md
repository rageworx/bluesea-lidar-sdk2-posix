# BlueSea Lidar SDK2 for POSIX
This repository modified all codes for these models

* MinGW-W64 ( Windows )
* macOS ( WIP : since macOS 11, maybe )
* Linux ( WIP : all architectures )

## USB-LIDAR DESCRIPTION
Windows/linux SDK and Demo programs for lanhai 2d lidar

# HOW TO BUILD AND USE

## All platforms that supports pthread.

* Prerequisite: g++ and pthread.
* Make a symlink from `Makefile.${platform_type}`
    ```
	ln -s Makefile.mingw64 Makefile
	```
* Build
	```
	make
	make demo
	./demo config/xxxx.txt
	```

## Other
**Special instructions: Please refer to the corresponding product's documentation to select the specified configuration file, and ensure that the software can be called normally only after the hardware works normally.**

## config file from windows to linux ##
```
vim xxxx.txt
set ff=unix
```
