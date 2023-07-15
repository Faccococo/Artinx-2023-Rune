cmake_minimum_required(VERSION 3.16)
set (SOURCE
    "Detector.cpp"
    "main.cpp"
    "Binarizer.cpp"
)
find_package(OpenCV CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
add_executable(Rune ${SOURCE})
target_link_libraries(Rune PRIVATE ${OpenCV_LIBS})
target_link_libraries(Rune PRIVATE fmt::fmt)