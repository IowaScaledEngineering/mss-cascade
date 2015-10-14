v 20130925 2
T 42300 59600 9 10 1 0 0 6 3
Free-mo
Accessory
Bus (~16VAC)
N 46800 59000 46800 58800 4
N 46800 59900 46800 60100 4
N 48800 60100 50900 60100 4
T 66900 39100 9 10 1 0 0 0 1
Free-mo MSS-Compatible Intregrated Cascade Module
T 66700 38800 9 10 1 0 0 0 1
mss-cascade.sch
T 66900 38500 9 10 1 0 0 0 1
1
T 68400 38500 9 10 1 0 0 0 1
1
T 70600 38500 9 10 1 0 0 0 1
Nathan D. Holmes
T 70600 38800 9 10 1 0 0 0 1
$Revision: 82 $
T 42000 39100 9 10 1 0 0 2 3
Notes:
1) All caps X5R or X7R, 6.3V or better ceramic unless otherwise noted.

C 43100 59500 1 0 1 termblk2-1.sym
{
T 42100 60150 5 10 0 0 0 6 1
device=TERMBLK2
T 42700 60400 5 10 1 1 0 6 1
refdes=J3
T 43100 59500 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 42700 39400 1 0 0 hole-1.sym
{
T 42700 39400 5 10 0 1 0 0 1
device=HOLE
T 42900 40000 5 10 1 1 0 4 1
refdes=H1
T 42700 39400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 43200 39400 1 0 0 hole-1.sym
{
T 43200 39400 5 10 0 1 0 0 1
device=HOLE
T 43400 40000 5 10 1 1 0 4 1
refdes=H2
T 43200 39400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 43700 39400 1 0 0 hole-1.sym
{
T 43700 39400 5 10 0 1 0 0 1
device=HOLE
T 43900 40000 5 10 1 1 0 4 1
refdes=H3
T 43700 39400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 44200 39400 1 0 0 hole-1.sym
{
T 44200 39400 5 10 0 1 0 0 1
device=HOLE
T 44400 40000 5 10 1 1 0 4 1
refdes=H4
T 44200 39400 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
N 45100 60100 47200 60100 4
N 48000 58800 48000 59500 4
C 48900 59900 1 270 0 capacitor-1.sym
{
T 49600 59700 5 10 0 1 270 0 1
device=CAPACITOR
T 49200 59600 5 10 1 1 0 0 1
refdes=C2
T 49800 59700 5 10 0 0 270 0 1
symversion=0.1
T 49200 59100 5 10 1 1 0 0 1
value=1uF
T 48900 59900 5 10 0 0 0 0 1
footprint=0805
}
N 49100 59900 49100 60100 4
N 49100 59000 49100 58800 4
C 41500 52500 1 0 0 current-transformer.sym
{
T 41900 53900 5 10 1 1 0 0 1
refdes=T1
T 41800 53800 5 10 0 0 0 0 1
device=transformer
T 41500 52500 5 10 0 0 0 0 1
footprint=CST306
}
N 42900 53700 46100 53700 4
{
T 45400 54100 5 10 1 1 0 0 1
netname=VBIAS
}
N 44700 53600 44700 53700 4
N 43300 53700 43300 53600 4
N 42900 52700 44900 52700 4
C 50700 60500 1 0 0 5V-plus-1.sym
N 50900 60500 50900 60100 4
C 57700 49900 1 0 0 mega48-tqfp32.sym
{
T 62200 56400 5 10 1 1 0 6 1
refdes=U2
T 58000 56700 5 10 0 0 0 0 1
device=ATMega48-TQFP32
T 58000 56900 5 10 0 0 0 0 1
footprint=TQFP32_7
}
N 59900 57200 64400 57200 4
C 61900 59100 1 0 0 avrprog-1.sym
{
T 61900 60700 5 10 0 1 0 0 1
device=AVRPROG
T 62500 60400 5 10 1 1 0 0 1
refdes=J3
T 61900 59100 5 10 0 0 0 0 1
footprint=JUMPER3x2
}
N 63300 60100 63600 60100 4
N 63600 60100 63600 60600 4
N 62500 50700 65200 50700 4
C 63200 58900 1 0 0 gnd-1.sym
N 63300 59300 63300 59200 4
C 60000 49300 1 0 0 gnd-1.sym
N 59900 49900 59900 49600 4
N 59900 49600 60300 49600 4
N 60300 49900 60300 49600 4
C 65300 51300 1 90 0 resistor-1.sym
{
T 64900 51600 5 10 0 0 90 0 1
device=RESISTOR
T 65000 51500 5 10 1 1 90 0 1
refdes=R18
T 65500 51500 5 10 1 1 90 0 1
value=10k
T 65300 51300 5 10 0 0 90 0 1
footprint=0805
}
C 65000 50500 1 270 0 capacitor-1.sym
{
T 65700 50300 5 10 0 1 270 0 1
device=CAPACITOR
T 65300 50200 5 10 1 1 0 0 1
refdes=C9
T 65900 50300 5 10 0 0 270 0 1
symversion=0.1
T 65300 49700 5 10 1 1 0 0 1
value=1uF
T 65000 50500 5 10 0 0 0 0 1
footprint=0805
}
C 65100 49100 1 0 0 gnd-1.sym
C 63500 54600 1 0 0 gnd-1.sym
N 62500 54900 63600 54900 4
N 65200 49600 65200 49400 4
C 63400 55800 1 270 0 capacitor-1.sym
{
T 64100 55600 5 10 0 1 270 0 1
device=CAPACITOR
T 63700 55500 5 10 1 1 0 0 1
refdes=C10
T 64300 55600 5 10 0 0 270 0 1
symversion=0.1
T 63700 55000 5 10 1 1 0 0 1
value=0.1uF
T 63400 55800 5 10 0 0 0 0 1
footprint=0805
}
N 62500 55500 62500 55800 4
N 60300 57200 60300 56600 4
N 62500 55200 63000 55200 4
N 63000 55200 63000 57200 4
N 65200 50500 65200 51300 4
N 62500 55800 63600 55800 4
C 46600 59900 1 270 0 Cap_H-2.sym
{
T 46900 59600 5 10 1 1 0 0 1
refdes=C1
T 48100 59900 5 10 0 0 270 0 1
device=Capacitor
T 46100 59600 5 10 1 1 0 2 1
value=68uF
T 46600 59900 5 10 0 1 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 45500 59100 5 10 1 1 0 0 1
description=35V Electrolytic
}
C 47200 59500 1 0 0 lm7805-1.sym
{
T 48800 60800 5 10 0 0 0 0 1
device=7805
T 48600 60500 5 10 1 1 0 6 1
refdes=U1
T 47200 59500 5 10 0 1 0 0 1
footprint=TO220
}
C 40800 38200 0 0 0 title-bordered-A1.sym
C 44800 52700 1 90 0 resistor-1.sym
{
T 44400 53000 5 10 0 0 90 0 1
device=RESISTOR
T 44500 53100 5 10 1 1 90 0 1
refdes=R9
T 45000 53100 5 10 1 1 90 0 1
value=1k
T 44800 52700 5 10 0 0 90 0 1
footprint=0805
}
N 45000 53700 45000 54200 4
N 45000 54200 45300 54200 4
N 45800 52700 46800 52700 4
N 46100 52700 46100 53300 4
N 47700 52700 48400 52700 4
N 48400 52700 48400 53500 4
N 48400 53500 49000 53500 4
C 49400 52600 1 0 0 gnd-1.sym
N 49000 53100 49000 51900 4
N 50200 51900 50200 53300 4
N 47900 51900 47700 51900 4
{
T 47100 51800 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 53300 52300 53300 4
{
T 52400 53200 5 10 1 1 0 0 1
netname=IDET1
}
C 51700 52100 1 0 0 gnd-1.sym
C 49300 53700 1 0 0 5V-plus-1.sym
N 62500 52500 63000 52500 4
{
T 63200 52400 5 10 1 1 0 0 1
netname=IDET1
}
N 62500 52200 63000 52200 4
{
T 63200 52100 5 10 1 1 0 0 1
netname=IDET2
}
C 44500 52600 1 0 0 res-pack4-1.sym
{
T 44500 52600 5 10 0 0 0 0 1
slot=2
T 45300 52400 5 10 1 1 0 0 1
value=10k
T 44700 52200 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 45000 52400 5 10 1 1 0 0 1
refdes=R5
}
C 48100 52600 1 0 1 res-pack4-1.sym
{
T 48100 52600 5 10 0 0 0 6 1
slot=1
T 47300 52400 5 10 1 1 0 6 1
value=10k
T 47900 52200 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 47600 52400 5 10 1 1 0 6 1
refdes=R5
}
C 46100 53100 1 0 0 tsv914-1.sym
{
T 46925 53250 5 8 0 0 0 0 1
device=TSV914
T 46400 53400 5 10 1 1 0 0 1
refdes=U4
T 45400 53400 5 10 1 1 0 0 1
device=TSV914
T 46100 53100 5 10 0 0 0 0 1
footprint=SO14
T 46100 53100 5 10 0 0 0 0 1
slot=4
}
C 49000 52900 1 0 0 tsv914-1.sym
{
T 49825 53050 5 8 0 0 0 0 1
device=TSV914
T 49300 53200 5 10 0 1 0 0 1
slot=3
T 49300 53200 5 10 1 1 0 0 1
refdes=U4
T 49000 52900 5 10 0 0 0 0 1
footprint=SO14
}
C 46400 55200 1 0 0 5V-plus-1.sym
C 46500 52800 1 0 0 gnd-1.sym
N 50300 53300 50000 53300 4
N 48800 51900 49200 51900 4
C 47500 51800 1 0 0 res-pack2-1.sym
{
T 47995 51600 5 10 1 1 0 0 1
refdes=R6
T 47500 51800 5 10 0 0 0 0 1
slot=1
T 47700 51400 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 48400 51600 5 10 1 1 0 0 1
value=1k
}
C 48800 51800 1 0 0 res-pack2-1.sym
{
T 49395 51600 5 10 1 1 0 0 1
refdes=R7
T 48800 51800 5 10 0 0 0 0 1
slot=1
T 49700 51600 5 10 1 1 0 0 1
value=100k
T 49100 51400 5 10 1 1 0 0 1
footprint=RPACK2-0606
}
N 50100 51900 50200 51900 4
C 42900 53600 1 270 0 mmbd4448dw-1.sym
{
T 43500 53200 5 10 0 0 270 0 1
device=MMBD4448DW
T 43200 52500 5 10 1 1 0 0 1
refdes=D3
T 42898 53605 5 10 0 1 270 0 1
footprint=SOT363
T 42900 53600 5 10 0 0 0 0 1
slot=1
}
C 43500 52700 1 270 1 mmbd4448dw-1.sym
{
T 44300 53900 5 10 1 1 180 2 1
device=MMBD4448DW
T 43800 52600 5 10 1 1 180 6 1
refdes=D3
T 43498 52695 5 10 0 1 90 2 1
footprint=SOT363
T 43500 52700 5 10 0 0 180 6 1
slot=2
}
N 43900 53600 43900 53700 4
C 47500 53100 1 0 0 mmbd4448dw-1.sym
{
T 47900 53700 5 10 0 0 0 0 1
device=MMBD4448DW
T 47900 54000 5 10 1 1 0 0 1
refdes=D4
T 47495 53098 5 10 0 1 0 0 1
footprint=SOT363
T 47500 53100 5 10 0 0 0 0 1
slot=1
}
N 47500 53500 47100 53500 4
C 49900 53200 1 0 0 res-pack4-1.sym
{
T 49900 53200 5 10 0 0 0 0 1
slot=1
T 50800 53800 5 10 1 1 0 0 1
value=10k
T 50200 53600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 53800 5 10 1 1 0 0 1
refdes=R8
}
C 41500 48900 1 0 0 current-transformer.sym
{
T 41800 50200 5 10 0 0 0 0 1
device=transformer
T 41500 48900 5 10 0 0 0 0 1
footprint=CST306
T 41900 50300 5 10 1 1 0 0 1
refdes=T2
}
N 42900 50100 46100 50100 4
N 44700 50000 44700 50100 4
N 43300 50100 43300 50000 4
N 42900 49100 44900 49100 4
C 44800 49100 1 90 0 resistor-1.sym
{
T 44400 49400 5 10 0 0 90 0 1
device=RESISTOR
T 44800 49100 5 10 0 0 90 0 1
footprint=0805
T 44500 49500 5 10 1 1 90 0 1
refdes=R10
T 45000 49500 5 10 1 1 90 0 1
value=1k
}
N 45000 50100 45000 50600 4
N 45000 50600 45400 50600 4
{
T 45500 50500 5 10 1 1 0 0 1
netname=VBIAS
}
N 45800 49100 46800 49100 4
N 46100 49100 46100 49700 4
N 47700 49100 48400 49100 4
N 48400 49100 48400 49900 4
N 48400 49900 49000 49900 4
C 49400 49000 1 0 0 gnd-1.sym
N 49000 49500 49000 48300 4
N 50200 48300 50200 49700 4
N 47900 48300 47700 48300 4
{
T 47100 48200 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 49700 52300 49700 4
{
T 52400 49600 5 10 1 1 0 0 1
netname=IDET2
}
C 51700 48500 1 0 0 gnd-1.sym
C 49300 50100 1 0 0 5V-plus-1.sym
C 44500 49000 1 0 0 res-pack4-1.sym
{
T 44500 49000 5 10 0 0 0 0 1
slot=4
T 45300 48800 5 10 1 1 0 0 1
value=10k
T 44700 48600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 45000 48800 5 10 1 1 0 0 1
refdes=R5
}
C 48100 49000 1 0 1 res-pack4-1.sym
{
T 48100 49000 5 10 0 0 0 6 1
slot=3
T 47300 48800 5 10 1 1 0 6 1
value=10k
T 47900 48600 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 47600 48800 5 10 1 1 0 6 1
refdes=R5
}
C 46100 49500 1 0 0 tsv914-1.sym
{
T 46925 49650 5 8 0 0 0 0 1
device=TSV914
T 46400 49800 5 10 1 1 0 0 1
refdes=U4
T 45400 49800 5 10 1 1 0 0 1
device=TSV914
T 46100 49500 5 10 0 1 0 0 1
slot=1
T 46100 49500 5 10 0 0 0 0 1
footprint=SO14
}
C 49000 49300 1 0 0 tsv914-1.sym
{
T 49825 49450 5 8 0 0 0 0 1
device=TSV914
T 49300 49600 5 10 0 1 0 0 1
slot=2
T 49300 49600 5 10 1 1 0 0 1
refdes=U4
T 49000 49300 5 10 0 0 0 0 1
footprint=SO14
}
C 46500 49200 1 0 0 gnd-1.sym
N 50300 49700 50000 49700 4
N 48800 48300 49200 48300 4
C 47500 48200 1 0 0 res-pack2-1.sym
{
T 47500 48200 5 10 0 0 0 0 1
slot=2
T 47995 48000 5 10 1 1 0 0 1
refdes=R6
T 47700 47800 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 48400 48000 5 10 1 1 0 0 1
value=1k
}
C 48800 48200 1 0 0 res-pack2-1.sym
{
T 48800 48200 5 10 0 0 0 0 1
slot=2
T 49395 48000 5 10 1 1 0 0 1
refdes=R7
T 49700 48000 5 10 1 1 0 0 1
value=100k
T 49100 47800 5 10 1 1 0 0 1
footprint=RPACK2-0606
}
N 50100 48300 50200 48300 4
C 42900 50000 1 270 0 mmbd4448dw-1.sym
{
T 43500 49600 5 10 0 0 270 0 1
device=MMBD4448DW
T 42898 50005 5 10 0 1 270 0 1
footprint=SOT363
T 43200 48900 5 10 1 1 0 0 1
refdes=D5
T 42900 50000 5 10 0 0 0 0 1
slot=1
}
C 43500 49100 1 270 1 mmbd4448dw-1.sym
{
T 44300 50300 5 10 1 1 180 2 1
device=MMBD4448DW
T 43498 49095 5 10 0 1 90 2 1
footprint=SOT363
T 43500 49100 5 10 0 0 180 6 1
slot=2
T 43800 49000 5 10 1 1 180 6 1
refdes=D5
}
N 43900 50000 43900 50100 4
C 47500 49500 1 0 0 mmbd4448dw-1.sym
{
T 47900 50100 5 10 0 0 0 0 1
device=MMBD4448DW
T 47495 49498 5 10 0 1 0 0 1
footprint=SOT363
T 47900 50400 5 10 1 1 0 0 1
refdes=D4
T 47500 49500 5 10 0 1 0 0 1
slot=2
}
N 47500 49900 47100 49900 4
C 49900 49600 1 0 0 res-pack4-1.sym
{
T 49900 49600 5 10 0 0 0 0 1
slot=2
T 50800 50200 5 10 1 1 0 0 1
value=10k
T 50200 50000 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 50200 5 10 1 1 0 0 1
refdes=R8
}
C 46400 50300 1 0 0 5V-plus-1.sym
C 44000 45200 1 0 0 gnd-1.sym
N 43400 45500 44100 45500 4
N 44100 45500 44100 45900 4
N 43600 46400 42800 46400 4
{
T 42500 46500 5 10 1 1 0 0 1
netname=ISET1
}
C 43900 47000 1 0 0 5V-plus-1.sym
N 44100 47000 44100 46800 4
C 47000 45200 1 0 0 gnd-1.sym
N 46400 45500 47100 45500 4
N 47100 45500 47100 45900 4
N 46600 46400 45800 46400 4
{
T 45500 46500 5 10 1 1 0 0 1
netname=ISET2
}
C 46900 47000 1 0 0 5V-plus-1.sym
N 47100 47000 47100 46800 4
C 63400 60600 1 0 0 5V-plus-1.sym
N 59900 56600 59900 57200 4
C 59700 57200 1 0 0 5V-plus-1.sym
N 57700 54600 55000 54600 4
{
T 54400 54600 5 10 1 1 0 0 1
netname=MISO
}
N 57700 54300 55000 54300 4
{
T 54500 54300 5 10 1 1 0 0 1
netname=SCK
}
N 61400 60100 61900 60100 4
{
T 60800 60000 5 10 1 1 0 0 1
netname=MISO
}
N 61400 59700 61900 59700 4
{
T 60900 59600 5 10 1 1 0 0 1
netname=SCK
}
N 63700 59700 63300 59700 4
{
T 63800 59700 5 10 1 1 0 0 1
netname=MOSI
}
N 61900 59300 61400 59300 4
{
T 61300 59200 5 10 1 1 0 6 1
netname=\_RESET\_
}
N 65900 51000 65200 51000 4
{
T 65400 51100 5 10 1 1 0 0 1
netname=\_RESET\_
}
C 65000 52200 1 0 0 5V-plus-1.sym
C 42000 47700 1 90 0 led-3.sym
{
T 42000 47700 5 10 0 0 0 0 1
footprint=0805
T 42250 47950 5 10 1 1 90 0 1
device=RED
T 41450 48150 5 10 1 1 90 0 1
refdes=D3
}
N 41800 47700 42200 47700 4
{
T 42300 47600 5 10 1 1 0 0 1
netname=LDRV2
}
N 41800 48600 42200 48600 4
{
T 42300 48500 5 10 1 1 0 0 1
netname=LCOM
}
C 42100 51400 1 90 0 led-3.sym
{
T 42100 51400 5 10 0 0 0 0 1
footprint=0805
T 42350 51650 5 10 1 1 90 0 1
device=RED
T 41550 51850 5 10 1 1 90 0 1
refdes=D2
}
N 41900 51400 42300 51400 4
{
T 42400 51300 5 10 1 1 0 0 1
netname=LDRV1
}
N 41900 52300 42300 52300 4
{
T 42400 52200 5 10 1 1 0 0 1
netname=LCOM
}
N 57700 50700 57300 50700 4
{
T 57200 50600 5 10 1 1 0 6 1
netname=LCOM
}
C 54400 51200 1 0 0 resistor-1.sym
{
T 54700 51600 5 10 0 0 0 0 1
device=RESISTOR
T 54400 51500 5 10 1 1 0 0 1
refdes=R16
T 54900 51500 5 10 1 1 0 0 1
value=330
T 54400 51200 5 10 0 0 0 0 1
footprint=0805
}
N 55300 51300 57700 51300 4
N 54400 51300 54000 51300 4
{
T 53900 51200 5 10 1 1 0 6 1
netname=LDRV1
}
N 55300 51000 57700 51000 4
C 54400 50900 1 0 0 resistor-1.sym
{
T 54700 51300 5 10 0 0 0 0 1
device=RESISTOR
T 54400 50700 5 10 1 1 0 0 1
refdes=R17
T 54900 50700 5 10 1 1 0 0 1
value=330
T 54400 50900 5 10 0 0 0 0 1
footprint=0805
}
N 54400 51000 54000 51000 4
{
T 53900 50900 5 10 1 1 0 6 1
netname=LDRV2
}
C 63400 57200 1 270 0 capacitor-1.sym
{
T 64100 57000 5 10 0 1 270 0 1
device=CAPACITOR
T 63700 56900 5 10 1 1 0 0 1
refdes=C13
T 64300 57000 5 10 0 0 270 0 1
symversion=0.1
T 63700 56400 5 10 1 1 0 0 1
value=0.1uF
T 63400 57200 5 10 0 0 0 0 1
footprint=0805
}
C 64200 57200 1 270 0 capacitor-1.sym
{
T 64900 57000 5 10 0 1 270 0 1
device=CAPACITOR
T 64500 56900 5 10 1 1 0 0 1
refdes=C14
T 65100 57000 5 10 0 0 270 0 1
symversion=0.1
T 64500 56400 5 10 1 1 0 0 1
value=0.1uF
T 64200 57200 5 10 0 0 0 0 1
footprint=0805
}
N 63600 56300 64400 56300 4
C 64300 56000 1 0 0 gnd-1.sym
C 46900 55000 1 270 0 capacitor-1.sym
{
T 47600 54800 5 10 0 1 270 0 1
device=CAPACITOR
T 47200 54700 5 10 1 1 0 0 1
refdes=C18
T 47800 54800 5 10 0 0 270 0 1
symversion=0.1
T 47200 54200 5 10 1 1 0 0 1
value=0.1uF
T 46900 55000 5 10 0 0 0 0 1
footprint=0805
}
N 46600 55200 46600 53900 4
N 46600 55000 47100 55000 4
C 47000 53800 1 0 0 gnd-1.sym
C 44200 45900 1 90 0 pot-1.sym
{
T 43300 46700 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 43800 46500 5 10 1 1 90 0 1
refdes=R12
T 42700 46700 5 10 0 0 90 0 1
footprint=bourns3266
T 44300 46200 5 10 1 1 0 0 1
value=100k
}
C 47200 45900 1 90 0 pot-1.sym
{
T 46300 46700 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 46800 46500 5 10 1 1 90 0 1
refdes=R13
T 45700 46700 5 10 0 0 90 0 1
footprint=bourns3266
T 47300 46200 5 10 1 1 0 0 1
value=100k
}
C 52000 52000 1 90 0 cap-pack4-1.sym
{
T 51505 53000 5 10 1 1 180 0 1
refdes=C3
T 52000 52000 5 10 0 0 0 0 1
slot=1
T 52100 52700 5 10 1 1 0 0 1
value=1uF
T 52100 52500 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 52100 52300 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 52000 48400 1 90 0 cap-pack4-1.sym
{
T 51505 49400 5 10 1 1 180 0 1
refdes=C3
T 52000 48400 5 10 0 0 0 0 1
slot=2
T 52100 49100 5 10 1 1 0 0 1
value=1uF
T 52100 48900 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 52100 48700 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 43600 45100 1 90 0 cap-pack4-1.sym
{
T 43105 46100 5 10 1 1 180 0 1
refdes=C3
T 43600 45100 5 10 0 0 0 0 1
slot=3
T 42300 45900 5 10 1 1 0 0 1
value=1uF
T 41600 45300 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 41600 45100 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
C 46600 45100 1 90 0 cap-pack4-1.sym
{
T 46105 46100 5 10 1 1 180 0 1
refdes=C3
T 46600 45100 5 10 0 0 0 0 1
slot=4
T 45300 45900 5 10 1 1 0 0 1
value=1uF
T 44600 45300 5 10 1 1 0 0 1
device=CKCA43X5R0J105M100AA
T 44600 45100 5 10 1 1 0 0 1
footprint=RPACK4-1206
}
N 62500 54000 63000 54000 4
{
T 63700 53900 5 10 1 1 0 6 1
netname=ISET1
}
N 62500 53700 63000 53700 4
{
T 63700 53600 5 10 1 1 0 6 1
netname=ISET2
}
C 49600 40300 1 0 0 rj45-1.sym
{
T 49600 43200 5 10 0 0 0 0 1
device=RJ45
T 49000 40000 5 10 0 1 0 0 1
footprint=modular_8p8c_lp.fp
T 49600 42200 5 10 1 1 0 0 1
refdes=J1
T 49400 40000 5 10 1 1 0 0 1
description=MSS Bus West
}
C 69400 40500 1 0 1 rj45-1.sym
{
T 69400 43400 5 10 0 0 0 6 1
device=RJ45
T 70000 40200 5 10 0 1 0 6 1
footprint=modular_8p8c_lp.fp
T 69400 42400 5 10 1 1 0 6 1
refdes=J2
T 69600 40100 5 10 1 1 0 6 1
description=MSS Bus East
}
N 50500 41700 68500 41700 4
N 50500 41500 51200 41500 4
N 51200 41500 51200 41900 4
N 51200 41900 68500 41900 4
N 50500 41300 53600 41300 4
N 53600 41100 53600 44800 4
{
T 53800 42300 5 10 1 1 90 0 1
netname=MSS_W_OCC
}
N 53600 41100 68500 41100 4
N 50500 41100 53400 41100 4
N 53400 41100 53400 40900 4
N 53400 40900 57200 40900 4
N 57200 40900 57200 41300 4
N 57200 41300 68500 41300 4
N 68500 40900 57700 40900 4
N 57700 40900 57700 40700 4
N 57700 40700 50500 40700 4
N 68500 40700 58000 40700 4
N 58000 40700 58000 40500 4
N 58000 40500 50500 40500 4
N 50500 40900 52800 40900 4
N 68500 41500 52800 41500 4
N 52800 41500 52800 40900 4
C 51700 39700 1 0 0 gnd-1.sym
N 51800 40000 51800 41100 4
C 43900 60300 1 180 1 bridge-2.sym
{
T 44100 59300 5 10 1 1 180 6 1
refdes=U2
T 44100 59100 5 10 1 1 180 6 1
device=MB110S
T 44100 58900 5 10 0 0 180 6 1
symversion=0.1
T 43900 60300 5 10 0 0 0 0 1
footprint=MB1-PLCC4
}
N 43100 60100 43900 60100 4
N 43100 59700 43900 59700 4
N 43900 59700 43900 59600 4
C 45300 58500 1 0 0 gnd-1.sym
N 45100 59600 45400 59600 4
N 45400 59600 45400 58800 4
N 45400 58800 49100 58800 4
C 50300 58800 1 90 0 resistor-1.sym
{
T 49900 59100 5 10 0 0 90 0 1
device=RESISTOR
T 50300 58800 5 10 0 0 90 0 1
footprint=0805
T 50000 59000 5 10 1 1 90 0 1
refdes=R1
T 50500 59000 5 10 1 1 90 0 1
value=330
}
N 50200 59700 50200 60100 4
C 50400 57800 1 90 0 led-3.sym
{
T 50400 57800 5 10 0 0 0 0 1
footprint=0805
T 50650 57750 5 10 1 1 90 0 1
device=GREEN LED
T 49850 58250 5 10 1 1 90 0 1
refdes=D1
}
C 50100 57400 1 0 0 gnd-1.sym
N 50200 57700 50200 57800 4
N 50200 58700 50200 58800 4
C 42000 55200 1 0 0 mcp1702-1.sym
{
T 42900 56200 5 10 0 1 0 0 1
footprint=SOT23
T 43400 56200 5 10 1 1 0 6 1
refdes=U3
T 42900 56200 5 10 1 1 0 6 1
device=AP2120
}
C 42700 54400 1 0 0 gnd-1.sym
C 43700 55800 1 270 0 capacitor-1.sym
{
T 44400 55600 5 10 0 1 270 0 1
device=CAPACITOR
T 44600 55600 5 10 0 0 270 0 1
symversion=0.1
T 43700 55800 5 10 0 0 0 0 1
footprint=0805
T 44000 55500 5 10 1 1 0 0 1
refdes=C12
T 44000 55000 5 10 1 1 0 0 1
value=1uF
}
N 43600 55800 45400 55800 4
{
T 45600 55700 5 10 1 1 0 0 1
netname=VBIAS
}
N 42800 54700 42800 55200 4
N 42800 54900 44900 54900 4
C 45000 54900 1 90 0 resistor-1.sym
{
T 44600 55200 5 10 0 0 90 0 1
device=RESISTOR
T 45000 54900 5 10 0 0 90 0 1
footprint=0805
T 45400 55500 5 10 1 1 180 0 1
refdes=R23
T 45400 55200 5 10 1 1 180 0 1
value=330
}
T 44100 56200 9 10 1 0 0 0 2
Note: R23 prevents current pushed through the feedback networks of
 the op-amps from affecting the 1.2V reference rail
C 41400 56300 1 0 0 5V-plus-1.sym
N 42000 55800 41600 55800 4
N 41600 55800 41600 56300 4
C 49800 41700 1 0 0 EMBEDDEDAO4882-1.sym
[
B 51400 43400 1500 1700 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
P 51400 44800 51100 44800 1 0 1
{
T 51300 44850 5 8 1 1 0 6 1
pinnumber=1
T 51300 44850 5 8 0 0 0 6 1
pinseq=1
}
P 51400 44400 51100 44400 1 0 1
{
T 51300 44450 5 8 1 1 0 6 1
pinnumber=2
T 51300 44450 5 8 0 0 0 6 1
pinseq=2
}
P 51400 44000 51100 44000 1 0 1
{
T 51300 44050 5 8 1 1 0 6 1
pinnumber=3
T 51300 44050 5 8 0 0 0 6 1
pinseq=3
}
P 51400 43600 51100 43600 1 0 1
{
T 51300 43650 5 8 1 1 0 6 1
pinnumber=4
T 51300 43650 5 8 0 0 0 6 1
pinseq=4
}
P 52900 43600 53200 43600 1 0 1
{
T 52995 43645 5 8 1 1 0 0 1
pinnumber=5
T 53000 43550 5 8 0 0 180 6 1
pinseq=5
}
P 52900 44400 53200 44400 1 0 1
{
T 52995 44445 5 8 1 1 0 0 1
pinnumber=7
T 53000 44350 5 8 0 0 180 6 1
pinseq=7
}
P 52900 44800 53200 44800 1 0 1
{
T 52995 44845 5 8 1 1 0 0 1
pinnumber=8
T 53000 44750 5 8 0 0 180 6 1
pinseq=8
}
L 52002 44798 52002 44648 3 0 0 0 -1 -1
L 52302 44798 52302 44648 3 0 0 0 -1 -1
L 52152 44648 52152 44698 3 0 0 0 -1 -1
L 52102 44648 52202 44648 3 0 0 0 -1 -1
L 52152 44748 52152 44798 3 0 0 0 -1 -1
L 51952 44598 52352 44598 3 0 0 0 -1 -1
L 51902 44798 51902 44948 3 0 0 0 -1 -1
L 52402 44798 52402 44948 3 0 0 0 -1 -1
L 52402 44948 52202 44948 3 0 0 0 -1 -1
L 52202 44898 52202 44998 3 0 0 0 -1 -1
L 52102 44948 52202 44998 3 0 0 0 -1 -1
L 52202 44898 52102 44948 3 0 0 0 -1 -1
L 52102 44898 52102 44998 3 0 0 0 -1 -1
L 52102 44948 51902 44948 3 0 0 0 -1 -1
L 51952 44648 52052 44648 3 0 0 0 -1 -1
L 52252 44648 52352 44648 3 0 0 0 -1 -1
L 52152 44748 52102 44698 3 0 0 0 -1 -1
L 52102 44698 52202 44698 3 0 0 0 -1 -1
L 52202 44698 52152 44748 3 0 0 0 -1 -1
L 52152 44798 52002 44798 3 0 0 0 -1 -1
L 51400 44400 52152 44400 3 0 0 0 -1 -1
L 52151 44599 52151 44399 3 0 0 0 -1 -1
L 51400 44800 52000 44800 3 0 0 0 -1 -1
L 52300 44800 52900 44800 3 0 0 0 -1 -1
L 52700 44800 52700 44400 3 0 0 0 -1 -1
L 52900 44400 52700 44400 3 0 0 0 -1 -1
L 52002 43998 52002 43848 3 0 0 0 -1 -1
L 52302 43998 52302 43848 3 0 0 0 -1 -1
L 52152 43848 52152 43898 3 0 0 0 -1 -1
L 52102 43848 52202 43848 3 0 0 0 -1 -1
L 52152 43948 52152 43998 3 0 0 0 -1 -1
L 51952 43798 52352 43798 3 0 0 0 -1 -1
L 51902 43998 51902 44148 3 0 0 0 -1 -1
L 52402 43998 52402 44148 3 0 0 0 -1 -1
L 52402 44148 52202 44148 3 0 0 0 -1 -1
L 52202 44098 52202 44198 3 0 0 0 -1 -1
L 52102 44148 52202 44198 3 0 0 0 -1 -1
L 52202 44098 52102 44148 3 0 0 0 -1 -1
L 52102 44098 52102 44198 3 0 0 0 -1 -1
L 52102 44148 51902 44148 3 0 0 0 -1 -1
L 51952 43848 52052 43848 3 0 0 0 -1 -1
L 52252 43848 52352 43848 3 0 0 0 -1 -1
L 52152 43948 52102 43898 3 0 0 0 -1 -1
L 52102 43898 52202 43898 3 0 0 0 -1 -1
L 52202 43898 52152 43948 3 0 0 0 -1 -1
L 52152 43998 52002 43998 3 0 0 0 -1 -1
L 51400 43600 52152 43600 3 0 0 0 -1 -1
L 52151 43799 52151 43599 3 0 0 0 -1 -1
L 51400 44000 52000 44000 3 0 0 0 -1 -1
L 52300 44000 52900 44000 3 0 0 0 -1 -1
L 52700 44000 52700 43600 3 0 0 0 -1 -1
L 52900 43600 52700 43600 3 0 0 0 -1 -1
P 52900 44000 53200 44000 1 0 1
{
T 52995 44045 5 8 1 1 0 0 1
pinnumber=6
T 53000 43950 5 8 0 0 180 6 1
pinseq=6
}
T 50100 43700 5 10 0 0 0 0 1
device=PS2501-1
T 52300 45200 8 10 0 1 0 6 1
refdes=U?
T 50100 43900 8 10 0 0 0 0 1
T 0 0 9 10 0 0 0 0 1
T 49800 41700 9 10 0 0 0 0 1
copyright=2006 DJ Delorie
T 49800 41700 9 10 0 0 0 0 1
dist-license=GPL
T 49800 41700 9 10 0 0 0 0 1
use-license=unlimited
]
{
T 51500 43100 5 10 1 1 0 0 1
device=NCV8402AD
T 52200 45200 5 10 1 1 0 6 1
refdes=U5
}
C 50600 42700 1 0 0 gnd-1.sym
N 50700 43000 50700 44800 4
N 50700 44800 51100 44800 4
N 51100 44000 50700 44000 4
N 53200 41500 53200 44000 4
{
T 53100 42300 5 10 1 1 90 0 1
netname=MSS_E_OCC
}
N 53900 44800 53200 44800 4
N 53200 44400 53600 44400 4
C 54800 45200 1 180 0 mmbd4448dw-1.sym
{
T 54900 45500 5 10 1 1 180 0 1
device=MMBD4448DW
T 54805 45202 5 10 0 1 180 0 1
footprint=SOT363
T 54200 45100 5 10 1 1 0 0 1
refdes=D6
T 54800 45200 5 10 0 0 270 0 1
slot=1
}
C 54800 44400 1 180 0 mmbd4448dw-1.sym
{
T 54400 43800 5 10 0 0 180 0 1
device=MMBD4448DW
T 54805 44402 5 10 0 1 180 0 1
footprint=SOT363
T 54200 44300 5 10 1 1 0 0 1
refdes=D6
T 54800 44400 5 10 0 0 270 0 1
slot=2
}
N 53200 44000 53900 44000 4
N 51100 44400 48800 44400 4
{
T 47600 44400 5 10 1 1 0 0 1
netname=W_OCC_EN
}
N 51100 43600 48800 43600 4
{
T 47600 43600 5 10 1 1 0 0 1
netname=E_OCC_EN
}
C 55300 44700 1 0 0 res-pack2-1.sym
{
T 55300 44700 5 10 0 0 0 0 1
slot=1
T 55795 44500 5 10 1 1 0 0 1
refdes=R2
T 55500 44300 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 56200 44500 5 10 1 1 0 0 1
value=2k
}
C 55300 43900 1 0 0 res-pack2-1.sym
{
T 55300 43900 5 10 0 0 0 0 1
slot=2
T 55795 43700 5 10 1 1 0 0 1
refdes=R2
T 55500 43500 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 56200 43700 5 10 1 1 0 0 1
value=2k
}
N 54800 44800 55700 44800 4
N 54800 44000 55700 44000 4
N 56600 44000 57000 44000 4
N 57000 44000 57000 45000 4
N 57000 44800 56600 44800 4
C 56800 45000 1 0 0 5V-plus-1.sym
N 55200 44000 55200 43300 4
N 55200 43300 56000 43300 4
{
T 56100 43200 5 10 1 1 0 0 1
netname=E_S_SENSE
}
N 55200 45800 55900 45800 4
{
T 56000 45700 5 10 1 1 0 0 1
netname=W_S_SENSE
}
N 55200 44800 55200 45800 4
N 57700 54900 55000 54900 4
{
T 54400 54900 5 10 1 1 0 0 1
netname=MOSI
}
C 59100 45200 1 180 0 mmbd4448dw-1.sym
{
T 59100 45500 5 10 1 1 180 0 1
device=MMBD4448DW
T 59105 45202 5 10 0 1 180 0 1
footprint=SOT363
T 59100 45200 5 10 0 0 270 0 1
slot=1
T 58500 45100 5 10 1 1 0 0 1
refdes=D7
}
C 59100 44400 1 180 0 mmbd4448dw-1.sym
{
T 58700 43800 5 10 0 0 180 0 1
device=MMBD4448DW
T 59105 44402 5 10 0 1 180 0 1
footprint=SOT363
T 59100 44400 5 10 0 0 270 0 1
slot=2
T 58500 44300 5 10 1 1 0 0 1
refdes=D7
}
C 59600 44700 1 0 0 res-pack2-1.sym
{
T 59600 44700 5 10 0 0 0 0 1
slot=1
T 60095 44500 5 10 1 1 0 0 1
refdes=R3
T 59800 44300 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 60500 44500 5 10 1 1 0 0 1
value=2k
}
C 59600 43900 1 0 0 res-pack2-1.sym
{
T 59600 43900 5 10 0 0 0 0 1
slot=2
T 60095 43700 5 10 1 1 0 0 1
refdes=R3
T 59800 43500 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 60500 43700 5 10 1 1 0 0 1
value=2k
}
N 59100 44800 60000 44800 4
N 59100 44000 60000 44000 4
N 60900 44000 61300 44000 4
N 61300 44000 61300 45000 4
N 61300 44800 60900 44800 4
C 61100 45000 1 0 0 5V-plus-1.sym
N 59500 44000 59500 43300 4
N 59500 43300 60300 43300 4
{
T 60400 43200 5 10 1 1 0 0 1
netname=E_AA_SENSE
}
N 59500 45800 60200 45800 4
{
T 60300 45700 5 10 1 1 0 0 1
netname=W_AA_SENSE
}
N 59500 44800 59500 45800 4
N 58200 44000 57900 44000 4
N 57900 42100 57900 44000 4
N 57900 42100 68500 42100 4
N 58200 44800 57600 44800 4
N 57600 44800 57600 42100 4
N 57600 42100 50900 42100 4
N 50500 41900 50900 41900 4
N 50900 41900 50900 42100 4
C 63800 45200 1 180 0 mmbd4448dw-1.sym
{
T 63800 45500 5 10 1 1 180 0 1
device=MMBD4448DW
T 63805 45202 5 10 0 1 180 0 1
footprint=SOT363
T 63800 45200 5 10 0 0 270 0 1
slot=1
T 63200 45100 5 10 1 1 0 0 1
refdes=D8
}
C 63800 44400 1 180 0 mmbd4448dw-1.sym
{
T 63400 43800 5 10 0 0 180 0 1
device=MMBD4448DW
T 63805 44402 5 10 0 1 180 0 1
footprint=SOT363
T 63800 44400 5 10 0 0 270 0 1
slot=2
T 63200 44300 5 10 1 1 0 0 1
refdes=D8
}
C 64300 44700 1 0 0 res-pack2-1.sym
{
T 64300 44700 5 10 0 0 0 0 1
slot=1
T 64795 44500 5 10 1 1 0 0 1
refdes=R4
T 64500 44300 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 65200 44500 5 10 1 1 0 0 1
value=2k
}
C 64300 43900 1 0 0 res-pack2-1.sym
{
T 64300 43900 5 10 0 0 0 0 1
slot=2
T 64795 43700 5 10 1 1 0 0 1
refdes=R4
T 64500 43500 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 65200 43700 5 10 1 1 0 0 1
value=2k
}
N 63800 44800 64700 44800 4
N 63800 44000 64700 44000 4
N 65600 44000 66000 44000 4
N 66000 44000 66000 45000 4
N 66000 44800 65600 44800 4
C 65800 45000 1 0 0 5V-plus-1.sym
N 64200 44000 64200 43300 4
N 64200 43300 65000 43300 4
{
T 65100 43200 5 10 1 1 0 0 1
netname=E_A_SENSE
}
N 64200 45800 64900 45800 4
{
T 65000 45700 5 10 1 1 0 0 1
netname=W_A_SENSE
}
N 64200 44800 64200 45800 4
N 62500 44000 62900 44000 4
N 62100 44800 62900 44800 4
N 62500 44000 62500 41900 4
N 62100 44800 62100 41700 4
N 55000 57100 57000 57100 4
{
T 54900 57200 5 10 1 1 180 0 1
netname=W_S_SENSE
}
N 57700 55800 57000 55800 4
N 57000 55800 57000 57100 4
N 55000 56800 56800 56800 4
{
T 54900 56900 5 10 1 1 180 0 1
netname=E_S_SENSE
}
N 56800 56800 56800 55500 4
N 56800 55500 57700 55500 4
N 55000 56500 56600 56500 4
{
T 54900 56400 5 10 1 1 0 6 1
netname=W_A_SENSE
}
N 56600 56500 56600 55200 4
N 56600 55200 57700 55200 4
N 55000 56200 56400 56200 4
{
T 54900 56100 5 10 1 1 0 6 1
netname=E_A_SENSE
}
N 56400 56200 56400 54900 4
N 55000 55900 56200 55900 4
{
T 54900 55800 5 10 1 1 0 6 1
netname=W_AA_SENSE
}
N 56200 55900 56200 54600 4
N 55000 55600 56000 55600 4
{
T 54900 55500 5 10 1 1 0 6 1
netname=E_AA_SENSE
}
N 56000 55600 56000 54300 4
N 57700 54000 55000 54000 4
{
T 53800 54000 5 10 1 1 0 0 1
netname=W_OCC_EN
}
N 57700 53700 55000 53700 4
{
T 53800 53700 5 10 1 1 0 0 1
netname=E_OCC_EN
}
C 49500 46200 1 0 0 res-pack2-1.sym
{
T 49500 46200 5 10 0 0 0 0 1
slot=1
T 49995 46000 5 10 1 1 0 0 1
refdes=R11
T 49700 45800 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 50400 46000 5 10 1 1 0 0 1
value=100k
}
C 49500 45400 1 0 0 res-pack2-1.sym
{
T 49500 45400 5 10 0 0 0 0 1
slot=2
T 49995 45200 5 10 1 1 0 0 1
refdes=R11
T 49700 45000 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 50400 45200 5 10 1 1 0 0 1
value=100k
}
N 49000 44400 49000 46300 4
N 49000 46300 49900 46300 4
N 49300 43600 49300 45500 4
N 49300 45500 49900 45500 4
N 50800 45500 51300 45500 4
N 51300 45500 51300 46300 4
N 52100 46300 50800 46300 4
C 52000 46000 1 0 0 gnd-1.sym
C 72100 59400 1 0 0 termblk2-1.sym
{
T 73100 60050 5 10 0 0 0 0 1
device=TERMBLK2
T 73000 59800 5 10 1 1 0 0 1
refdes=J4
T 72100 59400 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
}
C 72100 58600 1 0 0 termblk2-1.sym
{
T 73100 59250 5 10 0 0 0 0 1
device=TERMBLK2
T 73000 59000 5 10 1 1 0 0 1
refdes=J5
T 72100 58600 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
}
C 72100 57000 1 0 0 termblk2-1.sym
{
T 73100 57650 5 10 0 0 0 0 1
device=TERMBLK2
T 73000 57300 5 10 1 1 0 0 1
refdes=J7
T 72100 57000 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
}
C 72100 56200 1 0 0 termblk2-1.sym
{
T 73100 56850 5 10 0 0 0 0 1
device=TERMBLK2
T 73000 56500 5 10 1 1 0 0 1
refdes=J8
T 72100 56200 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
}
C 71600 60400 1 0 0 5V-plus-1.sym
N 72100 60000 71800 60000 4
N 71800 60000 71800 60400 4
C 71700 55900 1 0 0 gnd-1.sym
N 71800 56200 71800 56400 4
N 71800 56400 72100 56400 4
C 69800 59500 1 0 0 res-pack4-1.sym
{
T 69800 59500 5 10 0 0 0 0 1
slot=1
T 70700 60100 5 10 1 1 0 0 1
value=330
T 70100 59900 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 70200 60100 5 10 1 1 0 0 1
refdes=R14
}
C 69800 59100 1 0 0 res-pack4-1.sym
{
T 69800 59100 5 10 0 0 0 0 1
slot=2
T 70700 59700 5 10 0 1 0 0 1
value=10k
T 70100 59500 5 10 0 1 0 0 1
footprint=RPACK4-1206
T 70400 59700 5 10 0 1 0 0 1
refdes=R8
}
C 69800 58700 1 0 0 res-pack4-1.sym
{
T 69800 58700 5 10 0 0 0 0 1
slot=3
T 70700 59300 5 10 0 1 0 0 1
value=10k
T 70100 59100 5 10 0 1 0 0 1
footprint=RPACK4-1206
T 70400 59300 5 10 0 1 0 0 1
refdes=R8
}
N 71100 59600 72100 59600 4
N 71100 59200 72100 59200 4
N 71100 58800 72100 58800 4
C 69800 57900 1 0 0 res-pack4-1.sym
{
T 69800 57900 5 10 0 0 0 0 1
slot=1
T 70700 56500 5 10 1 1 0 0 1
value=330
T 70100 56300 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 70200 56500 5 10 1 1 0 0 1
refdes=R15
}
C 69800 57500 1 0 0 res-pack4-1.sym
{
T 69800 57500 5 10 0 0 0 0 1
slot=2
T 70700 58100 5 10 0 1 0 0 1
value=10k
T 70100 57900 5 10 0 1 0 0 1
footprint=RPACK4-1206
T 70400 58100 5 10 0 1 0 0 1
refdes=R8
}
C 69800 57100 1 0 0 res-pack4-1.sym
{
T 69800 57100 5 10 0 0 0 0 1
slot=3
T 70700 57700 5 10 0 1 0 0 1
value=10k
T 70100 57500 5 10 0 1 0 0 1
footprint=RPACK4-1206
T 70400 57700 5 10 0 1 0 0 1
refdes=R8
}
N 71100 58000 72100 58000 4
N 71100 57600 72100 57600 4
N 71100 57200 72100 57200 4
C 72100 57800 1 0 0 termblk2-1.sym
{
T 73100 58450 5 10 0 0 0 0 1
device=TERMBLK2
T 73000 58100 5 10 1 1 0 0 1
refdes=J6
T 72100 57800 5 10 0 1 0 6 1
footprint=TERMBLK2_200MIL
}
N 72100 58400 70000 58400 4
N 70000 58400 70000 58800 4
N 70200 59600 69700 59600 4
{
T 68900 59600 5 10 1 1 0 0 1
netname=W_GRN
}
N 70200 59200 69700 59200 4
{
T 68900 59200 5 10 1 1 0 0 1
netname=W_YLW
}
N 70200 58800 69700 58800 4
{
T 68900 58800 5 10 1 1 0 0 1
netname=W_RED
}
N 70200 58000 69700 58000 4
{
T 68900 58000 5 10 1 1 0 0 1
netname=E_GRN
}
N 70200 57600 69700 57600 4
{
T 68900 57600 5 10 1 1 0 0 1
netname=E_YLW
}
N 70200 57200 69700 57200 4
{
T 68900 57200 5 10 1 1 0 0 1
netname=E_RED
}
N 70000 56800 70000 57200 4
N 70000 56800 72100 56800 4
N 57700 52800 57200 52800 4
{
T 56400 52800 5 10 1 1 0 0 1
netname=W_GRN
}
N 57700 52500 57200 52500 4
{
T 56400 52500 5 10 1 1 0 0 1
netname=W_YLW
}
N 57700 52200 57200 52200 4
{
T 56400 52200 5 10 1 1 0 0 1
netname=W_RED
}
N 57700 51900 57200 51900 4
{
T 56400 51900 5 10 1 1 0 0 1
netname=E_GRN
}
N 57700 51600 57200 51600 4
{
T 56400 51600 5 10 1 1 0 0 1
netname=E_YLW
}
N 62500 51600 63000 51600 4
{
T 63800 51600 5 10 1 1 0 6 1
netname=E_RED
}
C 45300 43000 1 90 0 led-3.sym
{
T 45300 43000 5 10 0 0 0 0 1
footprint=0805
T 45550 43250 5 10 1 1 90 0 1
device=RED
T 44750 43450 5 10 1 1 90 0 1
refdes=D4
}
N 45100 43000 45500 43000 4
{
T 45600 42900 5 10 1 1 0 0 1
netname=LCOM
}
N 45100 43900 45500 43900 4
{
T 45600 43800 5 10 1 1 0 0 1
netname=LDRV1
}
C 72100 53600 1 0 0 termblk3-1.sym
{
T 73100 54250 5 10 0 0 0 0 1
device=HEADER3
T 72500 54900 5 10 1 1 0 0 1
refdes=J9
}
C 71800 53400 1 0 0 gnd-1.sym
N 71900 53700 71900 53800 4
N 71900 53800 72100 53800 4
C 71700 55300 1 0 0 5V-plus-1.sym
N 71900 55300 71900 54600 4
N 71900 54600 72100 54600 4
C 71300 54300 1 90 0 resistor-1.sym
{
T 70900 54600 5 10 0 0 90 0 1
device=RESISTOR
T 71000 54500 5 10 1 1 90 0 1
refdes=R16
T 71500 54500 5 10 1 1 90 0 1
value=10k
T 71300 54300 5 10 0 0 90 0 1
footprint=0805
}
N 71200 55200 71900 55200 4
N 64500 54100 72100 54200 4
{
T 69200 54300 5 10 1 1 0 0 1
netname=OS_OCC_OPTICAL
}
N 71200 54300 71200 54200 4
N 62500 51900 64500 51900 4
N 64500 51900 64500 54100 4
C 45300 41600 1 90 0 led-3.sym
{
T 45300 41600 5 10 0 0 0 0 1
footprint=0805
T 45550 41850 5 10 1 1 90 0 1
device=RED
T 44750 42050 5 10 1 1 90 0 1
refdes=D5
}
N 45100 41600 45500 41600 4
{
T 45600 41500 5 10 1 1 0 0 1
netname=LCOM
}
N 45100 42500 45500 42500 4
{
T 45600 42400 5 10 1 1 0 0 1
netname=LDRV2
}
C 68900 52300 1 0 1 5V-plus-1.sym
C 67700 52200 1 90 1 capacitor-1.sym
{
T 67000 52000 5 10 0 1 270 2 1
device=CAPACITOR
T 67400 51900 5 10 1 1 0 6 1
refdes=C2
T 66800 52000 5 10 0 0 270 2 1
symversion=0.1
T 67400 51400 5 10 1 1 0 6 1
value=1uF
T 67700 52200 5 10 0 0 0 6 1
footprint=0805
}
C 68700 46700 1 0 1 gnd-1.sym
C 67600 51000 1 0 1 gnd-1.sym
N 68700 52300 68700 51600 4
N 67500 52200 68700 52200 4
N 67700 48800 67700 49200 4
C 67800 48500 1 0 1 gnd-1.sym
C 72300 49100 1 0 1 switch-dip8-1.sym
{
T 70900 51675 5 8 0 0 0 6 1
device=SWITCH_DIP8
T 72000 51850 5 10 1 1 0 6 1
refdes=U?
}
N 72300 49300 72300 51600 4
C 72400 49000 1 0 1 gnd-1.sym
N 71000 49500 69600 49500 4
N 69600 49700 71000 49700 4
N 71000 49700 71000 49800 4
N 69600 49900 70800 49900 4
N 70800 49900 70800 50100 4
N 70800 50100 71000 50100 4
N 69600 50100 70600 50100 4
N 70600 50100 70600 50400 4
N 70600 50400 71000 50400 4
N 69600 50300 70400 50300 4
N 70400 50300 70400 50700 4
N 70400 50700 71000 50700 4
N 69600 50500 70200 50500 4
N 70200 50500 70200 51000 4
N 70200 51000 71000 51000 4
N 69600 50700 70000 50700 4
N 70000 50700 70000 51300 4
N 70000 51300 71000 51300 4
N 71000 51600 69800 51600 4
N 69800 51600 69800 50900 4
N 69800 50900 69600 50900 4
N 63700 48100 67700 48100 4
{
T 67200 47900 5 10 1 1 0 6 1
netname=SDA
}
N 63000 48300 67700 48300 4
{
T 67200 48400 5 10 1 1 0 6 1
netname=SCL
}
N 62500 51000 63400 51000 4
N 63400 51000 63400 48300 4
N 62500 51300 63700 51300 4
N 63700 47800 63700 51300 4
C 61700 48200 1 0 0 res-pack2-1.sym
{
T 61700 48200 5 10 0 0 0 0 1
slot=1
T 62295 48800 5 10 1 1 0 0 1
refdes=R17
T 62000 48600 5 10 1 1 0 0 1
footprint=RPACK2-0606
T 62600 48000 5 10 1 1 0 0 1
value=2k
}
C 61700 47700 1 0 0 res-pack2-1.sym
{
T 61700 47700 5 10 0 0 0 0 1
slot=2
T 62195 47500 5 10 0 1 0 0 1
refdes=R17
T 61900 47300 5 10 0 1 0 0 1
footprint=RPACK2-0606
T 62600 47500 5 10 1 1 0 0 1
value=2k
}
N 63000 47800 63700 47800 4
C 61500 48900 1 0 0 5V-plus-1.sym
N 61700 48900 61700 47800 4
N 61700 47800 62100 47800 4
N 62100 48300 61700 48300 4
C 67700 45800 1 0 0 pca9555-tssop24.sym
{
T 69400 51400 5 10 1 1 0 6 1
refdes=U6
T 68000 52600 5 10 0 0 0 0 1
device=ATMega328-TQFP32
T 68000 52800 5 10 0 0 0 0 1
footprint=TQFP32_7
}
