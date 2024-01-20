cd build
# cmake -j$(nproc) ..
make -j$(nproc)
echo "-- 编译完成"
cd ..
echo "-- 运行中"
./bin/yaml_test j$(nproc)
