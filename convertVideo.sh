if [ ! -f "./build/waifu2x-ncnn-vulkan"]
then
    echo "Please build waifu2x-ncnn-vulkan.  Exiting..."
	exit 0
fi

if [ ! -d "/proc/driver/nvidia/gpus/" ]
then
	echo "Directory /proc/driver/nvidia/gpus/ to doesn't exist.  waifu2x require cuda a cuda enabled device.  Exiting..."
	exit 0
fi

ls /proc/driver/nvidia/gpus/ | wc -l

exec ./build/waifu2x-ncnn-vulkan  -i "$1"  -o ./output -s 2 -g 0,1 -n 3 -v
exec ./encode.sh