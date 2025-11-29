idf_component_register(
        INCLUDE_DIRS "magic_enum/include/magic_enum"
)

target_compile_definitions(${COMPONENT_LIB} INTERFACE MAGIC_ENUM_RANGE_MIN=0)
target_compile_definitions(${COMPONENT_LIB} INTERFACE MAGIC_ENUM_RANGE_MAX=0xFF)