cmake -DCMAKE_BUILD_TYPE="release" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_COMMAND=cmake -S $(pwd) -B "build/Release" -G Ninja
cmake --build build/Release --parallel
arm-none-eabi-objcopy -O binary "build/Release/FlightComputer26.elf" "build/Release/FlightComputer26.bin" 


