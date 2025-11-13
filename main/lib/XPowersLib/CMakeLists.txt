
# MIT License

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRC_DIRS "src" "src/REG"
                       INCLUDE_DIRS "src"
                       REQUIRES arduino-esp32)

project(XPowersLib)
