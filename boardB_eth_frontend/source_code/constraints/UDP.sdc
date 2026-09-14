create_clock -name {clk_in} -period 20.000 -waveform {0.000 10.000} [get_ports {clk_50}]
create_clock -name {phy1_rgmii_rx_clk} -period 8.000 -waveform {0.000 4.000} [get_ports {phy1_rgmii_rx_clk}]

derive_clocks
rename_clock -name {pll_inst_125M_0} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[0]}]
rename_clock -name {pll_inst_125M_1} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[1]}]
#rename_clock -name {pll_inst_12p5M} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[2]}]
# Disabled: this PLL does not generate clkc[3] in the current configuration. The
# 2026-09-13 phy run reported USR-6001 "No clocks matched" plus USR-8130/8124/8159
# critical warnings for this line. Only clkc[1] (udp_clk, fanout 1824) and clkc[4]
# (fanout 1) exist, per final_timing.rpt's Clock Summary. Vendor leftover from a
# configuration that had a 25MHz output; harmless but it polluted every build log.
#rename_clock -name {pll_inst_25M} [get_clocks {u_clk_gen/u_pll_0/pll_inst.clkc[3]}]

create_generated_clock -name {udp_clk_125m} -source [get_pins {u_clk_gen/u_pll_0/pll_inst.clkc[1]}] -master_clock {pll_inst_125M_1} -divide_by 1.000 -phase 0.000 -add [get_nets {udp_clk}]
#create_generated_clock -name {udp_clk_12p5m} -add -source [get_pins {u_clk_gen/u_pll_0/pll_inst.clkc[2]}] -master_clock {pll_inst_12p5M} -divide_by 1.000 [get_nets {udp_clk}]
#create_generated_clock -name {udp_clk_1p25m} -add -source [get_pins {u_clk_gen/u_pll_0/pll_inst.clkc[2]}] -master_clock {pll_inst_12p5M} -divide_by 10.000 [get_nets {udp_clk}]

# DISABLED 2026-09-13 -- this line silently disarmed rx_byte_cdc's CDC FIFO.
#
# Evidence, not argument. The phy run of 2026-09-13 reported in
# UDP_EXAMPLE_exception.timing:
#     Total 48   Dominated 24   Shadowed 0   Ignored 24, by set_clock_groups
#     set_max_delay -from {*/primary_addr_gray_reg[*]} -to {*/sync_r1[*]} -datapath_only 7.700
# and final_timing.rpt's "Set Max Delay (Ignored)" group names all 24 paths as
#     u_rx_byte_cdc/u_rx_fifo/{wr_to_rd_cross_inst,rd_to_wr_cross_inst}/
#         primary_addr_gray_reg[*] -> sync_r1[*]
# crossing udp_clk_125m <-> u_pll_50/pll_inst.clkc[0]. So the vendor's IP-level skew
# bound on the gray-coded pointers of MY fifo_sdr_data_2 instance was being dropped.
# This is exactly what IPUG012 section 5 forbids, and exactly what Board A's
# src/user_source/constraints_source/timing.sdc section 6 keeps commented out for.
#
# An earlier comment here argued this was safe because the vendor's own
# rx_client_fifo.v crosses rgmii_rx_clk -> udp_clk under the equivalent -exclusive
# group for phy1_rgmii_rx_clk (still enabled below). That analogy was wrong and is
# recorded as wrong on purpose: rx_client_fifo uses a SINGLE-BIT toggle plus a 2-FF
# resync, which is immune to inter-bit skew, whereas fifo_sdr_data_2 crosses a
# MULTI-BIT gray pointer, whose correctness depends on the arrival skew of its bits
# being bounded -- which is why the IP ships a set_max_delay for it at all.
#
# Why commenting this out is safe for the vendor's own logic: the rgmii_rx_clk
# group below is what isolates the PHY-recovered clock, and it is independent of
# this line. udp_clk_125m is also just a second name for pll_inst_125M_1 (same tree,
# created -add on net udp_clk at divide_by 1), which is why the Clock Summary and
# the STNS total count its violations twice -- expect that double count when doing
# arithmetic on the report.
#
# What re-enables analysis here: every udp_clk <-> clk_50m path in this design goes
# through u_rx_byte_cdc/u_rx_fifo, so removing the group hands those crossings back
# to the IP's own 7.700ns bound instead of to nothing. If a future run reports a
# NON-FIFO udp_clk crossing as violated, constrain that pair specifically with
# set_max_delay -datapath_only, as Board A's sections 5.5/5.6 do -- do not restore
# this group.
#set_clock_groups -exclusive -group [get_clocks {udp_clk_125m}]
#set_clock_groups -exclusive -group [get_clocks {udp_clk_12p5m}]
#set_clock_groups -exclusive -group [get_clocks {udp_clk_1p25m}]


# ------------------------------------------------------------
# SDRAM hard-core DQ boundary exception
#
# The 2026-09-13 phy run measured, per path group:
#   u_sdram/u0_clk/pll_inst.clkc[2] -> clkc[1] : SWNS -6.699  STNS -210.704  32 endpoints
#   u_sdram/u0_clk/pll_inst.clkc[1] -> clkc[2] : SWNS -1.690  STNS  -43.037  32 endpoints
# together -253.741ns of the global STNS -367.210ns, i.e. 69%.
#
# These are phantom violations at the EG_PHY_SDRAM_2M_32 macro boundary, identical
# in shape to the ones Board A measured and then board-verified a fix for
# (timing.sdc section 5.5: -197.968 / -95.481, also 32+32 endpoints, also 6.453ns of
# pure cell delay, also "input (hard ip)" / "output (hard ip)" 5.000ns in the path).
# Every one of the 64 endpoints was enumerated and each is a hard-IP DQ pad or its
# output tri-state control:
#   read-in  : u_sdram/sdram.dq[n] -> u_sdram/sdram_syn_*.bpad, Logic Level 0,
#              Data Path Delay 100% cell / 0% net
#   write-out: u_sdram/u2_ram/u2_wrrd/app_wr_din_6d_reg[*] -> sdram_syn_*.ts ->
#              u_sdram/sdram.dq[n], Logic Level 1 (PAD=1), 92-95% cell
# Two begin points report under u3_udp_ip_protocol_stack/temac_data_process/
# temac_tx_sof_reg_syn_13 -- that name is a synthesis duplication artefact, not a
# real TEMAC path: its clock pin is net u_sdram/u1_app_wrrd/clk and it drives net
# u_sdram/u2_ram/u2_wrrd/sdr_t_dup_20 into sdram_syn_24.ts. Checked specifically
# because masking a genuine datapath here would have been the worst outcome.
#
# clkc[2] is the 180-degree-shifted clock the PHY uses to centre-align DQ sampling
# (Rise 4.000 / Fall 0.000 in an 8ns period); setup and hold there are guaranteed by
# Anlogic in silicon, but the tool statically budgets it as a 4ns half period, and
# Anlogic ships no .tcl with this IP (contrast the two async FIFOs, which both ship
# one and are mounted via settings.cfg's IpSDCList). There is no RTL to fix -- the
# boundary can only be expressed in SDC.
#
# Neither clock is a FIFO read or write clock, so this does not contravene the
# IPUG012 section 5 rule that the udp_clk_125m group above just broke.
# ------------------------------------------------------------
rename_clock -name {sdr_clk}     [get_clocks {u_sdram/u0_clk/pll_inst.clkc[1]}]
rename_clock -name {sdr_clk_sft} [get_clocks {u_sdram/u0_clk/pll_inst.clkc[2]}]
set_max_delay -from [get_clocks {sdr_clk}]     -to [get_clocks {sdr_clk_sft}] -datapath_only 100
set_max_delay -from [get_clocks {sdr_clk_sft}] -to [get_clocks {sdr_clk}]     -datapath_only 100


set_clock_groups -exclusive -group [get_clocks {phy1_rgmii_rx_clk}]
