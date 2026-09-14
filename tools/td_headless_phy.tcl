# ⚠️ SUPERSEDED (2026-09-14) — use tools/td_flow_exit.tcl instead.
#
#   This is the project-mode place & route companion to td_headless.tcl. The
#   run-directory flow (td_flow_exit.tcl run inside phy_1) is the one that
#   actually works and is what produced vision_sub_m5.bit. Kept only as the
#   record of why project mode was abandoned; see the banner in td_headless.tcl.
#
# Headless TD stage-2: launch place & route (phy_1) on a project whose syn_1
# already succeeded and was persisted. Companion to tools/td_headless.tcl.
#
# WHY SPLIT FROM td_headless.tcl
#   The combined run worked through synthesis, but td_commands_prompt.exe was
#   observed to exit between `wait_run syn_1` and `launch_runs phy_1` (main log
#   ends at "Run syn_1 success", no error printed). Running phy as a second
#   invocation sidesteps that: project state (syn_1 done) lives on disk in the
#   .al/runs, so phy_1 can simply be launched fresh.
#
# USAGE (same env vars as td_headless.tcl):
#   TD_PRJ=vision_sub_m1 \
#   TD_PRJ_DIR=D:/TD/26Anlu/src/vision_sub/td_project \
#   /d/TD/bin/td_commands_prompt.exe D:/TD/26Anlu/repo/tools/td_headless_phy.tcl \
#   > phy_stage.log 2>&1 < /dev/null

if { ![info exists ::env(TD_PRJ)] || ![info exists ::env(TD_PRJ_DIR)] } {
    puts "TD_HEADLESS_PHY: TD_PRJ and TD_PRJ_DIR must be set in the environment"
    exit 2
}
set prj    $::env(TD_PRJ)
set prjdir [string map {\\ /} $::env(TD_PRJ_DIR)]

puts "TD_HEADLESS_PHY: opening project $prjdir/$prj.al"
open_project "$prjdir/$prj.al"

if { [catch {reset_runs phy_1} msg] } {
    puts "TD_HEADLESS_PHY: reset_runs phy_1 -> $msg"
}

puts "TD_HEADLESS_PHY: launching place & route (phy_1)"
launch_runs phy_1
wait_run phy_1

puts "TD_HEADLESS_PHY: BUILD DONE - bitstream in $prjdir/${prj}_Runs/phy_1/"
exit 0
