
ffmpeg -r 30 -f image2 -s 1440x2560 -start_number 1 -i ./build/output/%0d.png -vcodec libx264 -crf 10  -pix_fmt yuv420p output.mp4
