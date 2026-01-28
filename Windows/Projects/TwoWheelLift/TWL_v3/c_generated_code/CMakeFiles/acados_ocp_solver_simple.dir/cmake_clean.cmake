file(REMOVE_RECURSE
  "acados_ocp_solver_simple.dll"
  "acados_ocp_solver_simple.dll.manifest"
  "acados_ocp_solver_simple.lib"
  "acados_ocp_solver_simple.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/acados_ocp_solver_simple.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
