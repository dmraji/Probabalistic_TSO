echo "Removing old build"
rm -rf CMakeFiles cmake_install.cmake CMakeCache.txt Makefile rc

echo "Building..."
cmake .

echo "Done! Now run $ make to compile."
