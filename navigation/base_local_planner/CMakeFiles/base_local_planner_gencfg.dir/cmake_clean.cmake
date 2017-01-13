FILE(REMOVE_RECURSE
  "CMakeFiles/base_local_planner_gencfg"
  "devel/include/base_local_planner/BaseLocalPlannerConfig.h"
  "devel/share/base_local_planner/docs/BaseLocalPlannerConfig.dox"
  "devel/share/base_local_planner/docs/BaseLocalPlannerConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/base_local_planner/cfg/BaseLocalPlannerConfig.py"
  "devel/share/base_local_planner/docs/BaseLocalPlannerConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/base_local_planner_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
