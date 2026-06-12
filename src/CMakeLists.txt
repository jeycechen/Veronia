idf_component_register(SRCS "ui/veronia_ui.cpp"
                            "motor/angleencoder.cpp"
                            "motor/as5047p_angle_test.cpp"
                            "motor/smartknob.cpp"
                            "motor/motor.cpp"
                    INCLUDE_DIRS "."
                                 "${PROJECT_DIR}/include"
                    REQUIRES
                        espressif__esp_simplefoc
                        esp_driver_spi
                        esp_timer
                        )
