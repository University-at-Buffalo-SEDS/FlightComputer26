cmake -DCMAKE_BUILD_TYPE="debug" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_COMMAND=cmake -S $(pwd) -B "build/Debug" -G Ninja
cmake --build build/Debug --parallel
arm-none-eabi-objcopy -O binary "build/Debug/FlightComputer26.elf" "build/Debug/FlightComputer26.bin" 


