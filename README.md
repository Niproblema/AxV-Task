## Requirements 
PCL 1.13.1 (All-in-One)

## Build
mkdir build
cd build
cmake ..
cmake --build . --config Release --target AxonV-Task
cp .\AxonV-Task\input.pcd .

## Run 
.\AxonV-Task\Release\AxonV-Task.exe