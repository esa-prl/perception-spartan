rock_library(spartan
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(spartan_bin Main.cpp
    DEPS spartan)

