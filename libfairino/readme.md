1. In Windows, create a build folder with the same level as cmakelist and run the following command in the build folder
cmake / cmake .. -A x64
make / cmake --build . --config debug
Or open CmakeList.txt in the libfairino folder through an IDE such as VisualStudio, Clion, etc
2. libfairino on linux supports compilation on x86 or Arm. Find the corresponding environment in the LinuxBuild folder
The script file is compiled by executing it with the sh command.