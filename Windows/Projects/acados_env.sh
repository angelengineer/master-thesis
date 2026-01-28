#! /usr/bin/bash

# --- 1. CONFIGURACIÓN FIJA (Ruta de instalación de Acados) ---
# Esta ruta es fija, no importa desde dónde llames el script
export ACADOS_INSTALL_DIR="/home/angel/Projects/acados"

# --- 2. CONFIGURACIÓN DE MATLAB ---
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/external/matlab-casadi/
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/interfaces/acados_matlab_octave/
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/interfaces/acados_matlab_octave/acados_template_mex/

# --- 3. LIBRERÍAS DE ACADOS (Core) ---
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib

# --- 4. LA PARTE MÁGICA (Dinámica) ---
# Aquí usamos $(pwd) para decir: 
# "Añade también cualquier carpeta de código generado que esté DONDE ESTOY AHORA MISMO"

# Busca carpetas que empiecen por "c_generated_code_" en el directorio actual
# y añádelas al LD_LIBRARY_PATH
CURRENT_DIR=$(pwd)
GENERATED_FOLDERS=$(find "$CURRENT_DIR" -maxdepth 1 -type d -name "c_generated_code_*")

if [ -z "$GENERATED_FOLDERS" ]; then
    echo "NOTA: No se detectaron carpetas de código generado (c_generated_code_*) aquí."
    echo "      Si generas código nuevo, recuerda volver a hacer 'source' o reiniciar MATLAB."
else
    for folder in $GENERATED_FOLDERS; do
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$folder
        echo "Añadida librería local: $folder"
    done
fi

echo "--------------------------------------------------"
echo " Entorno Acados cargado desde: $ACADOS_INSTALL_DIR"
echo " Listo para trabajar en: $CURRENT_DIR"
echo "--------------------------------------------------"