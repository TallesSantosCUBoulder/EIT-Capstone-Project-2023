(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: Active_Electrode-F_Cu.gbr_cutout_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Thursday, 14 March 2024 at 17:35)

(This preprocessor is used with a motion controller loaded with GRBL firmware.)
(It is configured to be compatible with almost any version of GRBL firmware.)

(TOOL DIAMETER: 0.5 mm)
(Feedrate_XY: 90.0 mm/min)
(Feedrate_Z: 60.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -2.75 mm)
(DepthPerCut: 0.25 mm <=>11 passes)
(Z_Move: 2.0 mm)
(Z Start: None mm)
(Z End: 15.0 mm)
(X,Y End: 0.0000, 0.0000 mm)
(Steps per circle: 64)
(Steps per circle: 64)
(Preprocessor Geometry: GRBL_11_no_M6)

(X range:   -0.3500 ...   30.5760  mm)
(Y range:  -60.0400 ...    0.3500  mm)

(Spindle Speed: 0.0 RPM)
G21
G90
G17
G94


G01 F90.00

M5             
G00 Z15.0000
G00 X0.0000 Y0.0000                
T1
(MSG, Change to Tool Dia = 0.5000)
M0
G00 Z15.0000
        
M03
G01 F90.00
G00 X-0.3500 Y-27.4950
G01 F60.00
G01 Z-0.2500
G01 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5760 Y-27.4950 F90.00
G00 X30.5760 Y-27.4950
G01 F60.00
G01 Z-0.5000
G01 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3500 Y-27.4950 F90.00
G00 X-0.3500 Y-27.4950
G01 F60.00
G01 Z-0.7500
G01 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5760 Y-27.4950 F90.00
G00 X30.5760 Y-27.4950
G01 F60.00
G01 Z-1.0000
G01 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3500 Y-27.4950 F90.00
G00 X-0.3500 Y-27.4950
G01 F60.00
G01 Z-1.2500
G01 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5760 Y-27.4950 F90.00
G00 X30.5760 Y-27.4950
G01 F60.00
G01 Z-1.5000
G01 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3500 Y-27.4950 F90.00
G00 X-0.3500 Y-27.4950
G01 F60.00
G01 Z-1.7500
G01 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5760 Y-27.4950 F90.00
G00 X30.5760 Y-27.4950
G01 F60.00
G01 Z-2.0000
G01 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3500 Y-27.4950 F90.00
G00 X-0.3500 Y-27.4950
G01 F60.00
G01 Z-2.2500
G01 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5760 Y-27.4950 F90.00
G00 X30.5760 Y-27.4950
G01 F60.00
G01 Z-2.5000
G01 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3500 Y-27.4950 F90.00
G00 X-0.3500 Y-27.4950
G01 F60.00
G01 Z-2.7500
G01 F90.00
G01 X-0.3500 Y-0.0000 F90.00
G01 X-0.3483 Y0.0343 F90.00
G01 X-0.3433 Y0.0683 F90.00
G01 X-0.3349 Y0.1016 F90.00
G01 X-0.3234 Y0.1339 F90.00
G01 X-0.3087 Y0.1650 F90.00
G01 X-0.2910 Y0.1944 F90.00
G01 X-0.2706 Y0.2220 F90.00
G01 X-0.2475 Y0.2475 F90.00
G01 X-0.2220 Y0.2706 F90.00
G01 X-0.1944 Y0.2910 F90.00
G01 X-0.1650 Y0.3087 F90.00
G01 X-0.1339 Y0.3234 F90.00
G01 X-0.1016 Y0.3349 F90.00
G01 X-0.0683 Y0.3433 F90.00
G01 X-0.0343 Y0.3483 F90.00
G01 X0.0000 Y0.3500 F90.00
G01 X30.2260 Y0.3500 F90.00
G01 X30.2603 Y0.3483 F90.00
G01 X30.2943 Y0.3433 F90.00
G01 X30.3276 Y0.3349 F90.00
G01 X30.3599 Y0.3234 F90.00
G01 X30.3910 Y0.3087 F90.00
G01 X30.4204 Y0.2910 F90.00
G01 X30.4480 Y0.2706 F90.00
G01 X30.4735 Y0.2475 F90.00
G01 X30.4966 Y0.2220 F90.00
G01 X30.5170 Y0.1944 F90.00
G01 X30.5347 Y0.1650 F90.00
G01 X30.5494 Y0.1339 F90.00
G01 X30.5609 Y0.1016 F90.00
G01 X30.5693 Y0.0683 F90.00
G01 X30.5743 Y0.0343 F90.00
G01 X30.5760 Y-0.0000 F90.00
G01 X30.5760 Y-27.4950 F90.00
G00 Z2.0000
G00 X30.5760 Y-31.9950
G01 F60.00
G01 Z-0.2500
G01 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3500 Y-31.9950 F90.00
G00 X-0.3500 Y-31.9950
G01 F60.00
G01 Z-0.5000
G01 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5760 Y-31.9950 F90.00
G00 X30.5760 Y-31.9950
G01 F60.00
G01 Z-0.7500
G01 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3500 Y-31.9950 F90.00
G00 X-0.3500 Y-31.9950
G01 F60.00
G01 Z-1.0000
G01 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5760 Y-31.9950 F90.00
G00 X30.5760 Y-31.9950
G01 F60.00
G01 Z-1.2500
G01 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3500 Y-31.9950 F90.00
G00 X-0.3500 Y-31.9950
G01 F60.00
G01 Z-1.5000
G01 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5760 Y-31.9950 F90.00
G00 X30.5760 Y-31.9950
G01 F60.00
G01 Z-1.7500
G01 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3500 Y-31.9950 F90.00
G00 X-0.3500 Y-31.9950
G01 F60.00
G01 Z-2.0000
G01 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5760 Y-31.9950 F90.00
G00 X30.5760 Y-31.9950
G01 F60.00
G01 Z-2.2500
G01 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3500 Y-31.9950 F90.00
G00 X-0.3500 Y-31.9950
G01 F60.00
G01 Z-2.5000
G01 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5760 Y-31.9950 F90.00
G00 X30.5760 Y-31.9950
G01 F60.00
G01 Z-2.7500
G01 F90.00
G01 X30.5760 Y-59.6900 F90.00
G01 X30.5743 Y-59.7243 F90.00
G01 X30.5693 Y-59.7583 F90.00
G01 X30.5609 Y-59.7916 F90.00
G01 X30.5494 Y-59.8239 F90.00
G01 X30.5347 Y-59.8550 F90.00
G01 X30.5170 Y-59.8844 F90.00
G01 X30.4966 Y-59.9120 F90.00
G01 X30.4735 Y-59.9375 F90.00
G01 X30.4480 Y-59.9606 F90.00
G01 X30.4204 Y-59.9810 F90.00
G01 X30.3910 Y-59.9987 F90.00
G01 X30.3599 Y-60.0134 F90.00
G01 X30.3276 Y-60.0249 F90.00
G01 X30.2943 Y-60.0333 F90.00
G01 X30.2603 Y-60.0383 F90.00
G01 X30.2260 Y-60.0400 F90.00
G01 X0.0000 Y-60.0400 F90.00
G01 X-0.0343 Y-60.0383 F90.00
G01 X-0.0683 Y-60.0333 F90.00
G01 X-0.1016 Y-60.0249 F90.00
G01 X-0.1339 Y-60.0134 F90.00
G01 X-0.1650 Y-59.9987 F90.00
G01 X-0.1944 Y-59.9810 F90.00
G01 X-0.2220 Y-59.9606 F90.00
G01 X-0.2475 Y-59.9375 F90.00
G01 X-0.2706 Y-59.9120 F90.00
G01 X-0.2910 Y-59.8844 F90.00
G01 X-0.3087 Y-59.8550 F90.00
G01 X-0.3234 Y-59.8239 F90.00
G01 X-0.3349 Y-59.7916 F90.00
G01 X-0.3433 Y-59.7583 F90.00
G01 X-0.3483 Y-59.7243 F90.00
G01 X-0.3500 Y-59.6900 F90.00
G01 X-0.3500 Y-31.9950 F90.00
G00 Z2.0000
M05
G00 Z2.0000
G00 Z15.00
G00 X0.0 Y0.0


