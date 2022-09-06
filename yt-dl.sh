if [ "$1" == "" ]
then
	echo "Missing URL"
	exit 1
fi

youtube-dl --external-downloader aria2c --external-downloader-args '-c -j 3 -x 3 -s 3 -k 1M' $1 
