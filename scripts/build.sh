cd build
cmake -j$(nproc) ..
make -j$(nproc)
echo "-- 编译完成"
