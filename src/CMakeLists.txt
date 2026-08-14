idf_component_register(SRCS "ui/veronia_ui.cpp"
                            "platform/spi_bus_manager.cpp"
                            "display/display.cpp"
                            "audio/audio_output.cpp"
                            "audio/microphone_array.cpp"
                            "storage/sd_storage.cpp"
                            "motor/angleencoder.cpp"
                            "motor/as5047p_angle_test.cpp"
                            "motor/smartknob.cpp"
                            "motor/motor.cpp"
                            "motor/open_loop_spin_test.cpp"
                    INCLUDE_DIRS "."
                                 "${PROJECT_DIR}/include"
                    REQUIRES
                        espressif__esp_simplefoc
                        esp_driver_spi
                        esp_driver_i2s
                        esp_lcd
                        esp_timer
                        fatfs
                        sdmmc
                        )
