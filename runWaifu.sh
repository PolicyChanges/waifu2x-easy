FILE=./build/waifu2x-ncnn-vulkan
if ! test -f "$FILE"; then
    echo "$FILE does not exist.  Try build first."
fi

#./prepareData.sh
while getopts ":h:r:d" option; do
   case $option in
		h) printf "Help: \n\t-r Removes all previous upscaling files. \n\t-d Downloads a youtube video from a given URL before upscaling\nexample:\n\t./runWaifu.sh -r -d https://youtube.com/blahblah\n";;
        r) rm -r ./build/output;;
		d) youtube-dl -o ./download.mp4 --external-downloader aria2c --external-downloader-args '-c -j 3 -x 3 -s 3 -k 1M' $2;;#${OPTARG};;
		\?) # Invalid option
         echo "Error: Invalid option"
         exit;;
	 esac
done

#mkdir -p ./build/output
#./build/waifu2x-ncnn-vulkan  -i ./download.mp4  -o ./build/output -s 2 -g 0,1 -n 3 -v
#./encode.sh
