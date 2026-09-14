# Headless TD: open a project once so TD creates its <prj>_Runs/ directories
# and the .prj snapshots, then exit.  Nothing is built here.
#
# WHY: the run-directory flow (tools/td_flow_exit.tcl + DefaultFlow.tcl inside
# syn_1/phy_1) needs those directories to exist, and TD only creates them when
# a project is opened.  Project-mode launch_runs/wait_run does create them, but
# it also leaves a half-built state and rewrites settings.cfg with empty
# device_name/package_name -- see doc/vision_sub/README.md section 6.2.  So we
# open, exit, patch settings.cfg, and then drive each stage directly.
#
# USAGE:
#   TD_PRJ=vision_sub_m5 \
#   TD_PRJ_DIR=D:/TD/26Anlu/src/vision_sub/td_project \
#   /d/TD/bin/td_commands_prompt.exe D:/TD/26Anlu/repo/tools/td_open_only.tcl

if { ![info exists ::env(TD_PRJ)] || ![info exists ::env(TD_PRJ_DIR)] } {
    puts "TD_OPEN_ONLY: TD_PRJ and TD_PRJ_DIR must be set in the environment"
    exit 2
}
set prj    $::env(TD_PRJ)
set prjdir [string map {\\ /} $::env(TD_PRJ_DIR)]

puts "TD_OPEN_ONLY: opening $prjdir/$prj.al"
open_project "$prjdir/$prj.al"
puts "TD_OPEN_ONLY: done (run directories and .prj snapshots created)"
exit 0
