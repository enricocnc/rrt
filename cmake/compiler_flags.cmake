function(add_compiler_flags TARGET)

	target_compile_options(${TARGET}
		PRIVATE $<$<CONFIG:Release>:-O3>
		PRIVATE $<$<CONFIG:Debug>:-O0 -pedantic -Wall -Wextra -Wconversion -Wsign-conversion -Wcast-align -Wcast-qual -Wformat=2 -Wlogical-op -Wold-style-cast -Wredundant-decls -Wshadow -Wunused>
	)

endfunction()