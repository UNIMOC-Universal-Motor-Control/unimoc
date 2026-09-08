add_unimoc_test(nvm_settings_test nvm_settings_test.cpp
    LIBRARIES unimoc_settings)
add_unimoc_test(settings_store_test settings_store_test.cpp
    LIBRARIES unimoc_settings)

add_custom_target(settings_tests DEPENDS
    nvm_settings_test
    settings_store_test)