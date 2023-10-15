add_subdirectory(misc)

add_subdirectory_ifdef(CONFIG_JTAG jtag)
add_subdirectory_ifdef(CONFIG_SENSOR sensor)
