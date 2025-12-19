cmake_minimum_required(VERSION 3.25)
project(KobukiTestApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Wątki są potrzebne, bo biblioteka z nich korzysta (nawet statyczna)
find_package(Threads REQUIRED)

# 2. Wskazujemy gdzie są pliki nagłówkowe (.h)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 3. Definiujemy naszą aplikację
add_executable(KobukiTest main.cpp)

# 4. Magia linkowania:
#    Wskazujemy dokładną ścieżkę do skopiowanego pliku .a
target_link_libraries(KobukiTest
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/EasyKobukiLibStatic.a
        Threads::Threads
)

# Opcjonalnie: flagi ostrzeżeń dla maina
if(UNIX)
    target_compile_options(KobukiTest PRIVATE -Wall -Wextra)
endif()