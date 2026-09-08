add_unimoc_test(node_identity_test node_identity_test.cpp
    LIBRARIES unimoc_cyphal)
add_unimoc_test(control_mode_test control_mode_test.cpp
    LIBRARIES unimoc_cyphal)
add_unimoc_test(cyphal_interface_test cyphal_interface_test.cpp
    LIBRARIES unimoc_cyphal)
add_unimoc_test(cyphal_transport_test cyphal_transport_test.cpp
    LIBRARIES unimoc_cyphal)

add_custom_target(cyphal_tests DEPENDS
    node_identity_test
    control_mode_test
    cyphal_interface_test
    cyphal_transport_test)