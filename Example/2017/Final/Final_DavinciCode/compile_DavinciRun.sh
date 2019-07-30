#!/bin/sh

gcc -Wall -I. -I./utils `pkg-config opencv --cflags` -I./include  -c -o davinci_main.o davinci_main.c

gcc -Wall -I. -I./utils `pkg-config opencv --cflags` -I./include  -c -o nvthread.o nvthread.c

gcc -Wall -I. `pkg-config opencv --cflags` -c -o ./include/davinci_ImageProcessing.o ./include/davinci_ImageProcessing.c

gcc -Wall -I. `pkg-config opencv --cflags` -c -o ./include/davinci_VehicleControl.o ./include/davinci_VehicleControl.c

gcc -Wall -o DavinciRun davinci_main.o nvthread.o ./include/davinci_ImageProcessing.o ./include/davinci_VehicleControl.o -L ./utils -lnvmedia -lnvtestutil_board -lnvtestutil_capture_input -lnvtestutil_i2c -lpthread `pkg-config opencv --libs`

