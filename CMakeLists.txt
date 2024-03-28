cmake_minimum_required(VERSION 3.27)
project(Reloj_Tesis)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(mainPrueba)

add_executable(Reloj_Tesis
        mainPrueba/mainPrueba.ino
)
