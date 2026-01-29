#!/bin/bash

ACADOS_DIR="/home/angel/Projects/acados"
PROJECT_DIR="$(pwd)"

echo "Limpiando y preparando directorios de compilación..."

# Función para limpiar y recrear (más seguro)
clean_and_create() {
    local dir="$1"
    if [ -d "$dir" ]; then
        echo "  Eliminando anterior: $dir"
        rm -rf "$dir"
    fi
    mkdir -p "$dir"
    echo "  Creado limpio: $dir"
}

# Variable para acumular paths
NEW_PATHS=""

# Verificar si hay archivos model_*.m
shopt -s nullglob  # Evita errores si no hay matches
model_files=(model_*.m)
shopt -u nullglob

if [ ${#model_files[@]} -eq 0 ]; then
    echo "AVISO: No se encontraron archivos model_*.m en $(pwd)"
else
    for model_file in "${model_files[@]}"; do
        # Extraer nombre: model_simplified.m -> model_simplified
        model_name="${model_file%.m}"
        gen_dir="c_generated_code_${model_name}"
        
        # Limpiar si existe, crear nuevo
        clean_and_create "$gen_dir"
        
        # Acumular path
        NEW_PATHS="$NEW_PATHS:$(pwd)/$gen_dir"
    done
fi

# Configurar entorno (añadir al LD_LIBRARY_PATH existente o crear nuevo)
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$NEW_PATHS:$PROJECT_DIR:$ACADOS_DIR/lib"
export ACADOS_INSTALL_DIR="$ACADOS_DIR"
export MATLABPATH="$MATLABPATH:$ACADOS_DIR/interfaces/acados_matlab_octave:$ACADOS_DIR/external/casadi-matlab"

echo ""
echo "Directorios listos en LD_LIBRARY_PATH:"
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep "c_generated_code" || echo "  (ninguno)"
echo ""
echo "Iniciando MATLAB con entorno limpio..."
matlab -desktop