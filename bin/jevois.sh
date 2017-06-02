#!/bin/sh

##############################################################################################################
# Default settings:
##############################################################################################################

CAMERA=ov9650
use_usbserial=1 # Use a serial-over-USB to communicate with JeVois command-line interface
use_usbsd=1     # Expose the JEVOIS partition of the microSD as a USB drive

if [ -f /boot/nousbserial ]; then use_usbserial=0; echo "JeVois serial-over-USB disabled"; fi
if [ -f /boot/nousbsd ]; then use_usbsd=0; echo "JeVois microSD access over USB disabled"; fi

# Block device we present to the host as a USB drive, or empty to not present it at start:
usbsdfile="/dev/mmcblk0p3"

if [ -f /boot/nousbsdauto ]; then usbsdfile=""; echo "JeVois microSD access over USB not AUTO"; fi

##############################################################################################################
# Fix Python-OpenCV library location if needed and pre-load Python and OpenCV so it is cached for faster startup
##############################################################################################################

if [ ! -f /usr/lib/python3.5/site-packages/cv2.so ]; then
    echo "Fixing OpenCV library location"
    # There is a flaw in the buildroot building of opencv, where the library file name still shows up as compiled for
    # whatever the host architecture was, even though it has been correctly cross-compiled for the JeVois ARM
    # processor. Here we fix that and also move the library to the python site-packages:
    mv /usr/python/3.5/cv2.cpython-35m-x86_64-linux-gnu.so /usr/lib/python3.5/site-packages/cv2.so
fi

# Run a stub python program whose purpose is to get the python3 and its cv2 module cached from microSD to memory:
#cat <<EOF > /tmp/pyopencvpreload.py
#import cv2
#import numpy as np
#EOF

#echo "Launching Python and Python-OpenCV pre-load..."
#python3 /tmp/pyopencvpreload.py & # run in the background while we complete our boot-up

##############################################################################################################
# Load all required kernel modules:
##############################################################################################################

cd /lib/modules/3.4.39

for m in videobuf-core videobuf-dma-contig videodev vfe_os vfe_subdev v4l2-common v4l2-int-device \
		       cci ${CAMERA} vfe_v4l2 ump disp mali ; do
    echo "### insmod ${m}.ko ###"
    insmod ${m}.ko
done

##############################################################################################################
# Install any new packages:
##############################################################################################################

cd /jevois
for f in packages/*.jvpkg; do
    if [ -f "${f}" ]; then
	echo "### Installing package ${f} ###"
	bzcat "${f}" | tar xvf -
	sync
	rm -f "${f}"
	sync
    fi
done

##############################################################################################################
# Find any newly unpacked postinstall scripts, run them, and delete them:
##############################################################################################################

for f in modules/*/*/postinstall; do
    if [ -f "${f}" ]; then
	echo "### Running ${f} ###"
	d=`dirname "${f}"`
	cd "${d}"
	sh postinstall
	sync
	rm -f postinstall
	sync
	cd /jevois
    fi
done

##############################################################################################################
# Build videomappings.cfg, if missing, from any info in the modules:
##############################################################################################################

if [ ! -f /jevois/config/videomappings.cfg ]; then
    cat /jevois/modules/*/*/videomappings.cfg > /jevois/config/videomappings.cfg
fi

##############################################################################################################
# Get a list of all our needed library paths:
##############################################################################################################

LIBPATH="/lib:/usr/lib"
for d in /jevois/lib/*; do if [ -d "${d}" ]; then LIBPATH="${LIBPATH}:${d}"; fi; done
export LD_LIBRARY_PATH=${LIBPATH}

##############################################################################################################
# Insert the gadget driver:
##############################################################################################################

echo "### Insert gadget driver ###"
MODES=`/usr/bin/jevois-module-param`

insmodopts=""
if [ "X${use_usbsd}" = "X1" -a "X${usbsdfile}" != "X" ]; then insmodopts="${insmodopts} file=${usbsdfile}"; fi

insmod /lib/modules/3.4.39/g_jevoisa33.ko modes=${MODES} use_serial=${use_usbserial} \
       use_storage=${use_usbsd} ${insmodopts}

##############################################################################################################
# Launch jevois-daemon:
##############################################################################################################

echo "### Start jevois daemon ###"
opts=""
if [ "X${use_usbserial}" != "X1" ]; then opts="${opts} --usbserialdev="; fi

# Finally start the jevois daemon:
/usr/bin/jevois-daemon ${opts}

