echo "Removing old build"
rm -rf CMakeFiles cmake_install.cmake CMakeCache.txt Makefile octoccupancy octoccupancy.bt

echo "Building..."
cmake -DCMAKE_BUILD_TYPE=Release .

echo "Done! Now run $ make to compile."
