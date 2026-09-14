# ⚠️ SUPERSEDED FOR BUILDS (2026-09-14) — but keep it for ONE job: creating run dirs.
#
#   Use tools/td_flow_exit.tcl (run-directory flow) for all real builds. Project
#   mode proved unreliable: results are not persisted, `reset_runs` cascades away
#   artefacts, and `wait_run` cannot tell that a child process died instantly, so
#   it waits forever. Measured failure: td_commands_prompt.exe exits between
#   `wait_run syn_1` and `launch_runs phy_1` with the log ending cleanly at
#   "Run syn_1 success" and no error anywhere.
#
#   What this script is STILL good for: it is the only way to (re)create
#   `<prj>_Runs/syn_1` + `phy_1` from the command line. TD writes those run
#   directories (with their settings.cfg and the `.prj` snapshot) itself on
#   first open -- the run-directory flow can only run *inside* dirs that already
#   exist. So: run this once to materialise the run dirs, then switch to
#   td_flow_exit.tcl for every subsequent build. See doc/vision_sub/README.md §6.2.
#
# Headless TD (TangDynasty) build driver in pure project-mode Tcl.
#
# WHY THIS EXISTS
#   tools/td_build.ps1 drives the *run-directory* flow (DefaultFlow.tcl inside
#   syn_1/phy_1 with a settings.cfg), which requires the run dirs that the TD
#   GUI creates on first open. This script instead uses TD's project-mode Tcl
#   API (SWUG105) and works straight from the .al file, with no GUI ever run:
#
#       open_project <prj>.al
#       launch_runs syn_1   -> wait_run syn_1
#       launch_runs phy_1   -> wait_run phy_1
#
#   TD writes the run directories (<prj>_Runs/syn_1, phy_1) next to the .al on
#   its own, so this is also how the run dirs get created for the first time.
#
# USAGE
#   Environment variables (NOT Tcl args -- td_commands_prompt.exe does not
#   forward argv):
#       TD_PRJ      project name = .al file name without extension
#                   (e.g. vision_sub_m1)
#       TD_PRJ_DIR  directory holding the .al (forward or back slashes both
#                   fine; we normalise to forward slashes for Tcl)
#
#   Run it:
#       TD_PRJ=vision_sub_m1 \
#       TD_PRJ_DIR=D:/TD/26Anlu/src/vision_sub/td_project \
#       /d/TD/bin/td_commands_prompt.exe D:/TD/26Anlu/repo/tools/td_headless.tcl
#
#   Any step failure raises a Tcl error; the final `exit` code therefore
#   reflects success (0) or failure (1). wait_run prints progress continuously.

if { ![info exists ::env(TD_PRJ)] || ![info exists ::env(TD_PRJ_DIR)] } {
    puts "TD_HEADLESS: TD_PRJ and TD_PRJ_DIR must be set in the environment"
    exit 2
}
set prj    $::env(TD_PRJ)
set prjdir [string map {\\ /} $::env(TD_PRJ_DIR)]

puts "TD_HEADLESS: opening project $prjdir/$prj.al"
open_project "$prjdir/$prj.al"

# A run left half-finished by a killed earlier attempt must be reset before it
# can be relaunched; a never-run project errors on reset_runs, hence the catch.
foreach r {syn_1 phy_1} {
    if { [catch {reset_runs $r} msg] } {
        puts "TD_HEADLESS: reset_runs $r -> $msg"
    }
}

puts "TD_HEADLESS: launching synthesis (syn_1)"
launch_runs syn_1
wait_run syn_1

puts "TD_HEADLESS: launching place & route (phy_1)"
launch_runs phy_1
wait_run phy_1

puts "TD_HEADLESS: BUILD DONE - bitstream should be in $prjdir/${prj}_Runs/phy_1/"
exit 0
