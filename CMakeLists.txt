cmake_minimum_required(VERSION 3.25)
project(EasyKobukiLibStatic)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Znajdź bibliotekę wątków (wymagane na Linux dla std::thread/sleep)
find_package(Threads REQUIRED)

# 2. Dodaj pliki źródłowe I NAGŁÓWKOWE (dobra praktyka dla IDE)
add_library(EasyKobukiLibStatic STATIC
        KobukiDriver.cpp
        KobukiDriver.h
)

# 3. Ustaw katalogi include, aby inne cele widziały KobukiDriver.h
target_include_directories(EasyKobukiLibStatic PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 4. Podlinkuj bibliotekę wątków
target_link_libraries(EasyKobukiLibStatic PRIVATE Threads::Threads)
set_target_properties(EasyKobukiLibStatic PROPERTIES PREFIX "")
# (Opcjonalnie) Dodaj flagi ostrzeżeń - bardzo przydatne przy portowaniu
if(UNIX)
    target_compile_options(EasyKobukiLibStatic PRIVATE -Wall -Wextra)
endif()
message(STATUS "Build output will be in: ${CMAKE_BINARY_DIR}")