
#!/bin/bash
# Batch conversion of raw files into bmp
# Uses the function raw2bmp

# the script should be called with one argument
if [ $# != 1 ]
then
  echo "Usage: $0 folder_with_raw_files"
  exit 1
fi

echo "$0 converts the raw files into bmp" 
n=0
# iterate over this files
for f in "$1"/*.raw
do
        # call a function to convert from raw to bmp format
        ./raw2bmp $f
	# increase counter
	n=$[ $n + 1 ] 
done

echo "$n files converted"

