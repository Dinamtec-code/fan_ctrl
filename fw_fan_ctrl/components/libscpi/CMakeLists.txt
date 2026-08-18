# Definimos la lista de archivos fuente explícitamente basada en tu Makefile
set(SRCS
    "src/error.c"
    "src/fifo.c"
    "src/ieee488.c"
    "src/minimal.c"
    "src/parser.c"
    "src/units.c"
    "src/utils.c"
    "src/lexer.c"
    "src/expression.c"
)

# Registramos el componente
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "inc"
    PRIV_INCLUDE_DIRS "src" # Agregamos src por los headers privados (lexer_private.h, etc.)
)

# Aplicamos los flags de compilación que tenías en CFLAGS del Makefile
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wextra
    -Wmissing-prototypes
    -Wimplicit
)

