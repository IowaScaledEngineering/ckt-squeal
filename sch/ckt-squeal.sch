v 20130925 2
T 55700 36100 9 10 1 0 0 0 1
Simple Sound Module
T 55600 35800 9 10 1 0 0 0 1
ckt-squeal.sch
T 55600 35500 9 10 1 0 0 0 1
1
T 57100 35500 9 10 1 0 0 0 1
1
T 59500 35500 9 10 1 0 0 0 1
Nathan D. Holmes
T 56100 37000 9 10 1 0 0 0 3
Notes:
1) All unpolarized capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
C 42900 40400 1 90 0 resistor-1.sym
{
T 42500 40700 5 10 0 0 90 0 1
device=RESISTOR
T 43200 40700 5 10 1 1 90 0 1
refdes=R8
T 43400 40600 5 10 1 1 90 0 1
value=1.0k
T 42900 40400 5 10 0 0 90 0 1
footprint=0805
}
C 42600 38200 1 270 0 capacitor-1.sym
{
T 43300 38000 5 10 0 1 270 0 1
device=CAPACITOR
T 42900 38100 5 10 1 1 0 0 1
refdes=C9
T 43500 38000 5 10 0 0 270 0 1
symversion=0.1
T 42900 37400 5 10 1 1 0 0 1
value=0.01uF
T 42600 38200 5 10 0 0 0 0 1
footprint=0805
T 42900 37200 5 10 1 1 0 0 1
description=16V NP0/C0G
}
N 42800 38200 42800 40400 4
C 42000 44500 1 0 1 avrprog-1.sym
{
T 42000 46100 5 10 0 1 0 6 1
device=AVRPROG
T 41400 44200 5 10 1 1 0 6 1
refdes=J3
T 42000 44500 5 10 0 0 0 0 1
footprint=JUMPER3x2
}
N 42000 45500 44100 45500 4
N 44100 45800 40300 45800 4
N 40300 45800 40300 45100 4
N 40300 45100 40600 45100 4
N 44100 45200 42600 45200 4
N 42600 45200 42600 45100 4
N 42600 45100 42000 45100 4
C 40500 44400 1 0 0 gnd-1.sym
C 46700 42500 1 0 0 gnd-1.sym
C 46400 47800 1 0 0 3.3V-plus-1.sym
C 40400 46100 1 0 0 3.3V-plus-1.sym
N 40600 45500 40600 46100 4
N 42000 42900 42000 44700 4
N 46600 42800 47600 42800 4
C 42300 39500 1 90 0 resistor-1.sym
{
T 41900 39800 5 10 0 0 90 0 1
device=RESISTOR
T 41700 39800 5 10 1 1 90 0 1
refdes=R7
T 42000 39700 5 10 1 1 90 0 1
value=200k
T 42300 39500 5 10 0 0 90 0 1
footprint=0805
}
C 42300 40400 1 90 0 resistor-1.sym
{
T 41900 40700 5 10 0 0 90 0 1
device=RESISTOR
T 41700 40800 5 10 1 1 90 0 1
refdes=R6
T 42000 40700 5 10 1 1 90 0 1
value=56.2k
T 42300 40400 5 10 0 0 90 0 1
footprint=0805
}
N 42200 39500 42800 39500 4
N 42200 41300 42200 45500 4
N 42800 41300 42800 44900 4
N 42800 44900 44100 44900 4
C 42700 37000 1 0 0 gnd-1.sym
C 45000 37200 1 0 0 gnd-1.sym
N 44600 38600 44600 37900 4
N 44600 37900 45600 37900 4
N 45600 37900 45600 38800 4
C 47400 36900 1 0 0 gnd-1.sym
N 47000 37400 47000 36700 4
C 48000 37400 1 0 0 Cap_H-2.sym
{
T 48000 38900 5 10 0 0 0 0 1
device=Capacitor
T 48600 37700 5 10 1 1 0 0 1
refdes=C10
T 48600 37500 5 10 1 1 0 2 1
value=1uF
T 48000 37400 5 10 0 0 0 0 1
footprint=cap-elec-Nichicon-WJ--D4.00-H5.40-mm
}
C 51100 38100 1 0 0 lm4864-1.sym
{
T 53100 39850 5 10 1 1 0 5 1
device=LM4864
T 52600 39750 5 10 1 1 0 6 1
refdes=U5
T 51100 38100 5 10 0 0 0 0 1
footprint=SO8
}
C 50500 38900 1 180 0 resistor-1.sym
{
T 50200 38500 5 10 0 0 180 0 1
device=RESISTOR
T 50000 38600 5 10 1 1 180 0 1
refdes=R10
T 50400 38600 5 10 1 1 180 0 1
value=16k
T 50500 38900 5 10 0 0 180 0 1
footprint=0805
}
N 51100 39100 50900 39100 4
N 50900 37600 50900 39100 4
N 50900 38500 51100 38500 4
N 50500 38800 51100 38800 4
N 49200 38800 49600 38800 4
N 50600 38800 50600 37800 4
N 50600 37800 52500 37800 4
N 53400 37300 53400 38800 4
N 53400 38800 53100 38800 4
C 52000 36300 1 0 0 gnd-1.sym
N 52100 36600 52100 38000 4
N 52100 41300 53600 41300 4
C 50700 37600 1 270 0 capacitor-1.sym
{
T 51400 37400 5 10 0 1 270 0 1
device=CAPACITOR
T 51000 37300 5 10 1 1 0 0 1
refdes=C12
T 51600 37400 5 10 0 0 270 0 1
symversion=0.1
T 51000 36800 5 10 1 1 0 0 1
value=1uF
T 50700 37600 5 10 0 0 0 0 1
footprint=0805
T 51400 36800 5 10 1 1 0 0 1
description=16V
}
N 50900 36700 52100 36700 4
C 54100 38500 1 0 0 termblk2-1.sym
{
T 55100 39150 5 10 0 0 0 0 1
device=TERMBLK2
T 54500 39400 5 10 1 1 0 0 1
refdes=J4
T 54100 38500 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
N 54100 38700 53400 38700 4
N 54100 39100 53100 39100 4
C 53500 40100 1 0 0 gnd-1.sym
N 46600 47800 46600 46600 4
N 46600 46600 47100 46600 4
C 44900 40400 1 0 0 5V-plus-1.sym
C 47300 38000 1 0 0 5V-plus-1.sym
N 45100 37500 45100 38400 4
N 44600 39000 42800 39000 4
C 44800 49800 1 0 0 lm7805-1.sym
{
T 45700 50800 5 10 1 1 0 0 1
device=7805
T 45300 50800 5 10 1 1 0 6 1
refdes=U1
T 44800 49800 5 10 0 0 0 0 1
footprint=TO220
}
C 51900 41800 1 0 0 5V-plus-1.sym
N 46400 50400 47900 50400 4
C 44100 50400 1 90 1 Cap_H-2.sym
{
T 43800 51100 5 10 1 1 180 6 1
refdes=C2
T 42600 50400 5 10 0 0 270 2 1
device=Capacitor
T 44200 50900 5 10 1 1 0 8 1
value=68uF
T 43800 50500 5 10 1 1 0 0 1
description=25V
T 44100 50400 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
}
C 53400 41300 1 270 0 capacitor-1.sym
{
T 54100 41100 5 10 0 1 270 0 1
device=CAPACITOR
T 53100 41000 5 10 1 1 0 0 1
refdes=C13
T 54300 41100 5 10 0 0 270 0 1
symversion=0.1
T 53000 40500 5 10 1 1 0 0 1
value=0.1uF
T 53400 41300 5 10 0 0 0 0 1
footprint=0805
T 53000 40300 5 10 1 1 0 0 1
description=16V
}
N 45100 40400 45100 39200 4
C 39500 35200 0 0 0 title-bordered-A2.sym
N 52100 39900 52100 41800 4
N 48000 37600 48000 36700 4
N 48000 36700 47000 36700 4
C 44600 38400 1 0 0 lmv358-1.sym
{
T 44475 38700 5 10 1 1 0 6 1
device=MCP6402-E
T 45300 39750 5 10 0 0 0 0 1
footprint=SO8
T 44500 39300 5 10 1 1 0 0 1
refdes=U3
T 44600 38400 5 10 0 0 0 0 1
slot=1
}
C 47000 37200 1 0 0 lmv358-1.sym
{
T 47675 37800 5 10 0 0 0 0 1
device=LMV358
T 47700 38550 5 10 0 0 0 0 1
footprint=SO8
T 46900 38100 5 10 1 1 0 0 1
refdes=U3
T 47000 37200 5 10 0 0 0 0 1
slot=2
}
C 40800 49800 1 0 1 termblk2-1.sym
{
T 39800 50450 5 10 0 0 0 6 1
device=TERMBLK2
T 40400 50700 5 10 1 1 0 6 1
refdes=J1
T 40800 49800 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
C 43100 50400 1 270 0 capacitor-1.sym
{
T 43800 50200 5 10 0 1 270 0 1
device=CAPACITOR
T 43100 50900 5 10 1 1 0 0 1
refdes=C1
T 44000 50200 5 10 0 0 270 0 1
symversion=0.1
T 43100 50700 5 10 1 1 0 0 1
value=10uF
T 43100 50400 5 10 0 0 0 0 1
footprint=0805
T 43100 50500 5 10 1 1 0 0 1
description=25V
}
C 47700 50400 1 0 0 5V-plus-1.sym
C 40700 48700 1 0 0 gnd-1.sym
N 40800 50000 40800 49000 4
N 40800 49500 53900 49500 4
N 45600 49500 45600 49800 4
C 46600 50400 1 270 0 capacitor-1.sym
{
T 47300 50200 5 10 0 1 270 0 1
device=CAPACITOR
T 46600 50900 5 10 1 1 0 0 1
refdes=C4
T 47500 50200 5 10 0 0 270 0 1
symversion=0.1
T 46600 50700 5 10 1 1 0 0 1
value=1uF
T 46600 50400 5 10 0 0 0 0 1
footprint=0805
T 46600 50500 5 10 1 1 0 0 1
description=25V
}
C 60100 52600 1 180 0 dm3at.sym
{
T 59800 48550 5 10 0 0 180 0 1
device=SD Memory Card
T 58800 51000 5 10 0 1 180 0 1
footprint=hirose-dm3at
T 60400 50700 5 10 1 1 180 0 1
refdes=J2
}
C 56400 47300 1 0 0 gnd-1.sym
N 56900 49100 58400 49100 4
C 56300 48500 1 270 0 capacitor-1.sym
{
T 57000 48300 5 10 0 1 270 0 1
device=CAPACITOR
T 55700 48100 5 10 1 1 0 0 1
refdes=C8
T 57200 48300 5 10 0 0 270 0 1
symversion=0.1
T 55700 47900 5 10 1 1 0 0 1
value=0.1uF
T 56300 48500 5 10 0 0 0 0 1
footprint=0805
T 56800 47800 5 10 0 1 0 0 1
description=10uF
}
N 58400 48500 56500 48500 4
N 57900 48800 58400 48800 4
{
T 57200 48700 5 10 1 1 0 0 1
netname=SDCLK
}
N 50800 45200 50300 45200 4
{
T 50900 45200 5 10 1 1 180 7 1
netname=SDCLK
}
N 57900 49400 58400 49400 4
{
T 57400 49300 5 10 1 1 0 0 1
netname=SDO
}
N 50800 45800 50300 45800 4
{
T 50900 45800 5 10 1 1 180 7 1
netname=SDO
}
N 57900 48200 58400 48200 4
{
T 57400 48100 5 10 1 1 0 0 1
netname=SDI
}
N 50800 45500 50300 45500 4
{
T 50900 45500 5 10 1 1 180 7 1
netname=SDI
}
N 57900 47900 58400 47900 4
{
T 57200 47800 5 10 1 1 0 0 1
netname=SDCS
}
N 43900 46300 43900 45800 4
{
T 43600 46400 5 10 1 1 0 0 1
netname=SDCS
}
C 57800 49900 1 0 0 gnd-1.sym
N 57900 50200 58400 50200 4
N 57900 50500 58400 50500 4
{
T 57200 50400 5 10 1 1 0 0 1
netname=SDDET
}
N 50800 44900 50300 44900 4
{
T 50900 44900 5 10 1 1 180 7 1
netname=SDDET
}
C 47600 50400 1 90 1 Cap_H-2.sym
{
T 47300 51100 5 10 1 1 180 6 1
refdes=C5
T 46100 50400 5 10 0 0 270 2 1
device=Capacitor
T 47700 50900 5 10 1 1 0 8 1
value=120uF
T 47200 50500 5 10 1 1 0 0 1
description=6.3V
T 47600 50400 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
}
C 51700 50900 1 270 0 capacitor-1.sym
{
T 52400 50700 5 10 0 1 270 0 1
device=CAPACITOR
T 52100 50700 5 10 1 1 0 0 1
refdes=C7
T 52600 50700 5 10 0 0 270 0 1
symversion=0.1
T 52100 50500 5 10 1 1 0 0 1
value=10uF
T 51700 50900 5 10 0 0 0 0 1
footprint=0805
}
C 51700 50900 1 0 0 3.3V-plus-1.sym
C 55200 43200 1 0 0 cpdt6-5v4.sym
{
T 57900 45100 5 10 1 1 0 6 1
refdes=U6
T 55400 45600 5 10 0 0 0 0 1
footprint=SOT26
T 55200 45100 5 10 1 1 0 0 1
device=CPDT6-5V4
}
C 59700 44500 1 0 0 termblk2-1.sym
{
T 60700 45150 5 10 0 0 0 0 1
device=TERMBLK2
T 60600 44800 5 10 1 1 0 0 1
refdes=J5
T 59700 44500 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
C 59700 42900 1 0 0 termblk2-1.sym
{
T 60700 43550 5 10 0 0 0 0 1
device=TERMBLK2
T 60600 43300 5 10 1 1 0 0 1
refdes=J7
T 59700 42900 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
C 59700 43700 1 0 0 termblk2-1.sym
{
T 60700 44350 5 10 0 0 0 0 1
device=TERMBLK2
T 60600 44100 5 10 1 1 0 0 1
refdes=J6
T 59700 43700 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
N 59700 44300 59700 43900 4
N 59700 44100 58100 44100 4
C 58800 43800 1 0 0 gnd-1.sym
C 54000 43800 1 0 0 gnd-1.sym
N 54100 44100 55200 44100 4
N 58100 43600 58600 43600 4
N 58600 43600 58600 45700 4
{
T 58500 46300 5 10 1 1 270 0 1
netname=SW2
}
N 58600 44700 59700 44700 4
N 59700 45100 59100 45100 4
N 59100 45100 59100 45700 4
{
T 59000 46300 5 10 1 1 270 0 1
netname=SW1
}
N 59700 43500 59000 43500 4
N 59000 42000 59000 43500 4
{
T 58900 42000 5 10 1 1 90 0 1
netname=SW3
}
N 59700 43100 59400 43100 4
N 59400 43100 59400 42500 4
N 59400 42500 54400 42500 4
N 54400 42100 54400 44600 4
{
T 54300 42100 5 10 1 1 90 0 1
netname=SW4
}
N 54400 44600 55200 44600 4
N 50800 44600 50300 44600 4
{
T 50900 44600 5 10 1 1 180 7 1
netname=SW1
}
N 50800 44300 50300 44300 4
{
T 50900 44300 5 10 1 1 180 7 1
netname=SW2
}
N 50800 44000 50300 44000 4
{
T 50900 44000 5 10 1 1 180 7 1
netname=SW3
}
N 50800 43700 50300 43700 4
{
T 50900 43700 5 10 1 1 180 7 1
netname=SW4
}
C 49400 41000 1 0 1 led-3.sym
{
T 48450 41650 5 10 0 0 0 6 1
device=LED
T 48950 41550 5 10 1 1 0 6 1
refdes=D4
T 49400 41000 5 10 0 0 0 0 1
footprint=1206
}
C 49400 40200 1 0 1 led-3.sym
{
T 48450 40850 5 10 0 0 0 6 1
device=LED
T 48950 40750 5 10 1 1 0 6 1
refdes=D5
T 49400 40200 5 10 0 0 0 0 1
footprint=1206
}
C 49400 41800 1 0 1 led-3.sym
{
T 48450 42450 5 10 0 0 0 6 1
device=LED
T 48950 42350 5 10 1 1 0 6 1
refdes=D3
T 49400 41800 5 10 0 0 0 0 1
footprint=1206
}
C 49400 41900 1 0 0 resistor-1.sym
{
T 49700 42300 5 10 0 0 0 0 1
device=RESISTOR
T 49500 41600 5 10 1 1 0 0 1
refdes=R3
T 49900 41600 5 10 1 1 0 0 1
value=330
T 49400 41900 5 10 0 0 0 0 1
footprint=0805
}
C 49400 41100 1 0 0 resistor-1.sym
{
T 49700 41500 5 10 0 0 0 0 1
device=RESISTOR
T 49500 40800 5 10 1 1 0 0 1
refdes=R4
T 49900 40800 5 10 1 1 0 0 1
value=330
T 49400 41100 5 10 0 0 0 0 1
footprint=0805
}
C 49400 40300 1 0 0 resistor-1.sym
{
T 49700 40700 5 10 0 0 0 0 1
device=RESISTOR
T 49500 40000 5 10 1 1 0 0 1
refdes=R5
T 49900 40000 5 10 1 1 0 0 1
value=330
T 49400 40300 5 10 0 0 0 0 1
footprint=0805
}
C 50200 39700 1 0 0 gnd-1.sym
N 50300 42000 50300 40000 4
N 43100 41300 43100 44600 4
N 42000 43700 44100 43700 4
C 40300 42900 1 90 0 resistor-1.sym
{
T 39900 43200 5 10 0 0 90 0 1
device=RESISTOR
T 40600 43200 5 10 1 1 90 0 1
refdes=R2
T 40800 43100 5 10 1 1 90 0 1
value=10k
T 40300 42900 5 10 0 0 90 0 1
footprint=0805
}
C 40000 43800 1 0 0 3.3V-plus-1.sym
N 40200 42900 42000 42900 4
N 43100 44600 44100 44600 4
N 44100 44300 43400 44300 4
N 43400 44300 43400 41600 4
N 43700 42000 48500 42000 4
N 43700 44000 44100 44000 4
N 41500 50400 40800 50400 4
N 42400 50400 44800 50400 4
C 49200 47500 1 0 1 led-3.sym
{
T 48250 48150 5 10 0 0 0 6 1
device=LED
T 48850 47250 5 10 1 1 0 6 1
refdes=D2
T 49200 47500 5 10 0 0 0 0 1
footprint=1206
}
C 49400 47600 1 0 0 resistor-1.sym
{
T 49700 48000 5 10 0 0 0 0 1
device=RESISTOR
T 49500 47200 5 10 1 1 0 0 1
refdes=R1
T 49900 47200 5 10 1 1 0 0 1
value=330
T 49400 47600 5 10 0 0 0 0 1
footprint=0805
}
C 50500 47400 1 0 0 gnd-1.sym
N 50300 47700 50600 47700 4
C 41500 50200 1 0 0 schottky-1.sym
{
T 41822 50872 5 10 0 0 0 0 1
device=DIODE
T 41800 50700 5 10 1 1 0 0 1
refdes=D1
T 42341 50532 5 10 0 1 0 0 1
footprint=SOD123
T 41600 49900 5 10 1 1 0 0 1
device=CDBM140
}
C 47400 47700 1 270 0 capacitor-1.sym
{
T 48100 47500 5 10 0 1 270 0 1
device=CAPACITOR
T 47700 47400 5 10 1 1 0 0 1
refdes=C14
T 48300 47500 5 10 0 0 270 0 1
symversion=0.1
T 47900 47000 5 10 1 1 0 0 1
value=0.1uF
T 47400 47700 5 10 0 0 0 0 1
footprint=0805
T 47900 46800 5 10 1 1 0 0 1
description=16V
}
N 46600 47700 48300 47700 4
C 47500 46500 1 0 0 gnd-1.sym
N 49200 47700 49400 47700 4
N 51100 39400 51100 40100 4
{
T 50900 40300 5 10 1 1 0 0 1
netname=SHDN
}
N 45600 38800 48300 38800 4
C 46500 36800 1 90 0 resistor-1.sym
{
T 46100 37100 5 10 0 0 90 0 1
device=RESISTOR
T 46200 37300 5 10 1 1 90 0 1
refdes=R11
T 46200 36900 5 10 1 1 90 0 1
value=10k
T 46500 36800 5 10 0 0 90 0 1
footprint=0805
}
C 46500 37900 1 90 0 resistor-1.sym
{
T 46100 38200 5 10 0 0 90 0 1
device=RESISTOR
T 46200 38400 5 10 1 1 90 0 1
refdes=R12
T 46200 38000 5 10 1 1 90 0 1
value=27k
T 46500 37900 5 10 0 0 90 0 1
footprint=0805
}
C 46300 36500 1 0 0 gnd-1.sym
N 46400 37900 46400 37700 4
N 46400 37800 47000 37800 4
C 52500 37900 1 180 1 pot-bourns.sym
{
T 53300 37000 5 10 0 0 180 6 1
device=VARIABLE_RESISTOR
T 53000 38000 5 10 1 1 0 3 1
refdes=R9
T 52500 37900 5 10 0 0 270 2 1
footprint=TO220
T 52800 37600 5 10 1 1 180 0 1
value=25k
}
N 53000 37300 53400 37300 4
N 42700 50400 42700 51300 4
N 42700 51300 53100 51300 4
C 40000 42900 1 270 0 capacitor-1.sym
{
T 40700 42700 5 10 0 1 270 0 1
device=CAPACITOR
T 40300 42600 5 10 1 1 0 0 1
refdes=C3
T 40900 42700 5 10 0 0 270 0 1
symversion=0.1
T 40500 42200 5 10 1 1 0 0 1
value=1uF
T 40000 42900 5 10 0 0 0 0 1
footprint=0805
T 40500 42000 5 10 1 1 0 0 1
description=16V
}
C 40100 41700 1 0 0 gnd-1.sym
T 43200 49200 9 10 1 0 0 0 1
Note:  C5 & C6 must be low ESR but greater than 10mOhm
C 45500 40200 1 270 0 capacitor-1.sym
{
T 46200 40000 5 10 0 1 270 0 1
device=CAPACITOR
T 45800 39900 5 10 1 1 0 0 1
refdes=C15
T 46400 40000 5 10 0 0 270 0 1
symversion=0.1
T 46000 39500 5 10 1 1 0 0 1
value=0.1uF
T 45500 40200 5 10 0 0 0 0 1
footprint=0805
T 46000 39300 5 10 1 1 0 0 1
description=16V
}
N 43700 42000 43700 44000 4
N 48500 41200 47700 41200 4
N 47700 41200 47700 41600 4
N 47700 41600 43400 41600 4
N 48500 40400 47300 40400 4
N 47300 40400 47300 41300 4
N 47300 41300 43100 41300 4
N 45100 40300 45700 40300 4
N 45700 40300 45700 40200 4
C 45600 39000 1 0 0 gnd-1.sym
C 49200 37000 1 0 0 header2-1.sym
{
T 50200 37650 5 10 0 0 0 0 1
device=HEADER2
T 49600 37900 5 10 1 1 0 0 1
refdes=J8
T 49200 37000 5 10 0 0 0 0 1
footprint=JUMPER2
}
C 48800 36900 1 0 0 gnd-1.sym
N 49200 37600 48900 37600 4
N 49200 37200 48900 37200 4
C 49200 39000 1 180 0 Cap_H-2.sym
{
T 49200 37500 5 10 0 0 180 0 1
device=Capacitor
T 48600 38700 5 10 1 1 180 0 1
refdes=C11
T 48600 38900 5 10 1 1 180 2 1
value=0.22uF
T 49200 39000 5 10 0 0 180 0 1
footprint=cap-elec-Nichicon-WJ--D4.00-H5.40-mm
}
C 56800 48800 1 0 0 gnd-1.sym
N 55200 43600 54800 43600 4
N 54800 43600 54800 45400 4
N 54800 45400 59100 45400 4
N 58100 44600 58300 44600 4
N 58300 44600 58300 42900 4
N 58300 42900 59000 42900 4
C 43800 42600 1 0 0 ATtiny861-mlf32.sym
{
T 48400 46400 5 10 1 1 0 0 1
refdes=U4
T 45700 46400 5 10 1 1 0 6 1
device=ATtiny861
T 46095 42995 5 10 1 1 0 8 1
footprint=QFN32_5x5_0.5
}
N 42900 46500 42900 45200 4
{
T 42600 46600 5 10 1 1 0 0 1
netname=SHDN
}
C 55800 50600 1 90 1 Cap_H-2.sym
{
T 55500 51300 5 10 1 1 180 6 1
refdes=C6
T 54300 50600 5 10 0 0 270 2 1
device=Capacitor
T 55700 51100 5 10 1 1 0 8 1
value=120uF
T 55400 50700 5 10 1 1 0 0 1
description=6.3V
T 55800 50600 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
}
N 56500 48500 56500 50600 4
N 56500 50600 54700 50600 4
C 52500 49000 1 0 0 gnd-1.sym
N 52600 49300 52600 49500 4
C 55500 49400 1 0 0 gnd-1.sym
N 53100 51300 53100 50600 4
C 48900 50000 1 0 0 lp2985-1.sym
{
T 49700 50150 5 10 1 1 0 5 1
device=LP2985-33
T 50200 51050 5 10 1 1 0 6 1
refdes=U2
T 48900 50000 5 10 0 0 0 0 1
footprint=SOT23-5
}
N 48900 50900 48900 51300 4
N 48900 50600 48500 50600 4
N 48500 50600 48500 49500 4
N 48900 50300 48700 50300 4
N 48700 50300 48700 51300 4
N 50500 50900 51900 50900 4
N 51900 50000 51900 49500 4
C 50900 50400 1 270 0 capacitor-1.sym
{
T 51600 50200 5 10 0 1 270 0 1
device=CAPACITOR
T 50300 49900 5 10 1 1 0 0 1
refdes=C16
T 51800 50200 5 10 0 0 270 0 1
symversion=0.1
T 50300 49700 5 10 1 1 0 0 1
value=0.01uF
T 50900 50400 5 10 0 0 0 0 1
footprint=0805
}
N 51100 50400 50500 50400 4
N 50500 50400 50500 50300 4
C 53100 50000 1 0 0 78l33-1.sym
{
T 54700 51300 5 10 0 0 0 0 1
device=7805
T 54500 51000 5 10 1 1 0 6 1
refdes=U7
T 53100 50000 5 10 0 0 0 0 1
footprint=SOT89
}
N 53900 50000 53900 49500 4
