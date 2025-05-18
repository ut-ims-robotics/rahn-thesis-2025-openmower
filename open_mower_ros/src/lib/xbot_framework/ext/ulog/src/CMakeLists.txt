# Add a library with the above sources
add_library(ulog EXCLUDE_FROM_ALL ulog.c)

target_include_directories(ulog PUBLIC .)
