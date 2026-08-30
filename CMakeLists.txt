if(CONFIG_SHIELD_TORABO_TSUKI_LP_LEFT OR CONFIG_SHIELD_TORABO_TSUKI_LP_RIGHT)
  zephyr_library()
  zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
  zephyr_library_sources(src/board.c src/mini_trackpad_init_reg.c)
  zephyr_library_sources_ifdef(CONFIG_TORABO_SPLIT_PERIPHERAL_ADV_RECOVERY
    src/split_peripheral_adv_recovery.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_ACCELERATION)
  zephyr_library_named(zmk-input-processor-scroll-acceleration)
  zephyr_library_sources(src/input_processor_scroll_acceleration.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_INERTIA)
  zephyr_library_named(zmk-input-processor-scroll-inertia)
  zephyr_library_sources(src/input_processor_scroll_inertia.c)
endif()
