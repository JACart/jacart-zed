#!/bin/bash
echo "Creating video from svo file"

if [ -z "$1" ]
then
    echo "No input filename provided exiting"
    exit 1
fi

fileName=$(date +"jcart_%m-%d-%Y_%T")

if [ -z "$2" ]
then
    echo "No file name provided defualting to $fileName"
else
    fileName=$2
    echo "with name: $fileName"
fi

fileName="${fileName}.avi"
svoName="${1}.svo"
wavName="${1}.wav"

# Create the video 
python3 svo_export.py $svoName bin/original_zed_video.avi 1

# flip vertically
# ffmpeg -i bin/original_zed_video.avi -vf "vflip" bin/flip_zed_video.avi

# crop
# "[0]crop=iw/2:ih:0:0"
ffmpeg -i bin/original_zed_video.avi -filter_complex "[0]crop=iw/2:ih:ow:0" bin/crop.avi 

# Combine wav and avi
ffmpeg -i bin/crop.avi -i $wavName -c:v copy -c:a aac $fileName

# clean up excess avi videos
rm -rf bin/*
