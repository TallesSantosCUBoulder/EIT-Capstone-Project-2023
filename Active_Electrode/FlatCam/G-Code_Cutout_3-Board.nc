(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: Combo_MultiGeo_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Tuesday, 19 March 2024 at 00:19)

(This preprocessor is used with a motion controller loaded with GRBL firmware.)
(It is configured to be compatible with almost any version of GRBL firmware.)

(TOOL DIAMETER: 0.5 mm)
(Feedrate_XY: 120.0 mm/min)
(Feedrate_Z: 60.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -2.5 mm)
(DepthPerCut: 0.5 mm <=>5 passes)
(Z_Move: 2.0 mm)
(Z Start: None mm)
(Z End: 15.0 mm)
(X,Y End: 0.0000, 0.0000 mm)
(Steps per circle: 64)
(Steps per circle: 64)
(Preprocessor Geometry: GRBL_11_no_M6)

(X range:   -0.3500 ...   80.2875  mm)
(Y range:  -59.0310 ...    0.3500  mm)

(Spindle Speed: 0.0 RPM)
G21
G90
G17
G94


G01 F120.00

M5             
G00 Z15.0000
G00 X0.0000 Y0.0000                
T1
(MSG, Change to Tool Dia = 0.5000)
M0
G00 Z15.0000
        
M03
G01 F120.00
G00 X12.8710 Y0.3500
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X0.0000 Y0.3500 F120.00
G01 X-0.0343 Y0.3483 F120.00
G01 X-0.0683 Y0.3433 F120.00
G01 X-0.1016 Y0.3349 F120.00
G01 X-0.1339 Y0.3234 F120.00
G01 X-0.1650 Y0.3087 F120.00
G01 X-0.1944 Y0.2910 F120.00
G01 X-0.2220 Y0.2706 F120.00
G01 X-0.2475 Y0.2475 F120.00
G01 X-0.2706 Y0.2220 F120.00
G01 X-0.2910 Y0.1944 F120.00
G01 X-0.3087 Y0.1650 F120.00
G01 X-0.3234 Y0.1339 F120.00
G01 X-0.3349 Y0.1016 F120.00
G01 X-0.3433 Y0.0683 F120.00
G01 X-0.3483 Y0.0343 F120.00
G01 X-0.3500 Y-0.0000 F120.00
G01 X-0.3500 Y-58.6810 F120.00
G01 X-0.3483 Y-58.7153 F120.00
G01 X-0.3433 Y-58.7493 F120.00
G01 X-0.3349 Y-58.7826 F120.00
G01 X-0.3234 Y-58.8149 F120.00
G01 X-0.3087 Y-58.8460 F120.00
G01 X-0.2910 Y-58.8754 F120.00
G01 X-0.2706 Y-58.9030 F120.00
G01 X-0.2475 Y-58.9285 F120.00
G01 X-0.2220 Y-58.9516 F120.00
G01 X-0.1944 Y-58.9720 F120.00
G01 X-0.1650 Y-58.9897 F120.00
G01 X-0.1339 Y-59.0044 F120.00
G01 X-0.1016 Y-59.0159 F120.00
G01 X-0.0683 Y-59.0243 F120.00
G01 X-0.0343 Y-59.0293 F120.00
G01 X0.0000 Y-59.0310 F120.00
G01 X12.8710 Y-59.0310 F120.00
G00 X12.8710 Y-59.0310
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X0.0000 Y-59.0310 F120.00
G01 X-0.0343 Y-59.0293 F120.00
G01 X-0.0683 Y-59.0243 F120.00
G01 X-0.1016 Y-59.0159 F120.00
G01 X-0.1339 Y-59.0044 F120.00
G01 X-0.1650 Y-58.9897 F120.00
G01 X-0.1944 Y-58.9720 F120.00
G01 X-0.2220 Y-58.9516 F120.00
G01 X-0.2475 Y-58.9285 F120.00
G01 X-0.2706 Y-58.9030 F120.00
G01 X-0.2910 Y-58.8754 F120.00
G01 X-0.3087 Y-58.8460 F120.00
G01 X-0.3234 Y-58.8149 F120.00
G01 X-0.3349 Y-58.7826 F120.00
G01 X-0.3433 Y-58.7493 F120.00
G01 X-0.3483 Y-58.7153 F120.00
G01 X-0.3500 Y-58.6810 F120.00
G01 X-0.3500 Y-0.0000 F120.00
G01 X-0.3483 Y0.0343 F120.00
G01 X-0.3433 Y0.0683 F120.00
G01 X-0.3349 Y0.1016 F120.00
G01 X-0.3234 Y0.1339 F120.00
G01 X-0.3087 Y0.1650 F120.00
G01 X-0.2910 Y0.1944 F120.00
G01 X-0.2706 Y0.2220 F120.00
G01 X-0.2475 Y0.2475 F120.00
G01 X-0.2220 Y0.2706 F120.00
G01 X-0.1944 Y0.2910 F120.00
G01 X-0.1650 Y0.3087 F120.00
G01 X-0.1339 Y0.3234 F120.00
G01 X-0.1016 Y0.3349 F120.00
G01 X-0.0683 Y0.3433 F120.00
G01 X-0.0343 Y0.3483 F120.00
G01 X0.0000 Y0.3500 F120.00
G01 X12.8710 Y0.3500 F120.00
G00 X12.8710 Y0.3500
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X0.0000 Y0.3500 F120.00
G01 X-0.0343 Y0.3483 F120.00
G01 X-0.0683 Y0.3433 F120.00
G01 X-0.1016 Y0.3349 F120.00
G01 X-0.1339 Y0.3234 F120.00
G01 X-0.1650 Y0.3087 F120.00
G01 X-0.1944 Y0.2910 F120.00
G01 X-0.2220 Y0.2706 F120.00
G01 X-0.2475 Y0.2475 F120.00
G01 X-0.2706 Y0.2220 F120.00
G01 X-0.2910 Y0.1944 F120.00
G01 X-0.3087 Y0.1650 F120.00
G01 X-0.3234 Y0.1339 F120.00
G01 X-0.3349 Y0.1016 F120.00
G01 X-0.3433 Y0.0683 F120.00
G01 X-0.3483 Y0.0343 F120.00
G01 X-0.3500 Y-0.0000 F120.00
G01 X-0.3500 Y-58.6810 F120.00
G01 X-0.3483 Y-58.7153 F120.00
G01 X-0.3433 Y-58.7493 F120.00
G01 X-0.3349 Y-58.7826 F120.00
G01 X-0.3234 Y-58.8149 F120.00
G01 X-0.3087 Y-58.8460 F120.00
G01 X-0.2910 Y-58.8754 F120.00
G01 X-0.2706 Y-58.9030 F120.00
G01 X-0.2475 Y-58.9285 F120.00
G01 X-0.2220 Y-58.9516 F120.00
G01 X-0.1944 Y-58.9720 F120.00
G01 X-0.1650 Y-58.9897 F120.00
G01 X-0.1339 Y-59.0044 F120.00
G01 X-0.1016 Y-59.0159 F120.00
G01 X-0.0683 Y-59.0243 F120.00
G01 X-0.0343 Y-59.0293 F120.00
G01 X0.0000 Y-59.0310 F120.00
G01 X12.8710 Y-59.0310 F120.00
G00 X12.8710 Y-59.0310
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X0.0000 Y-59.0310 F120.00
G01 X-0.0343 Y-59.0293 F120.00
G01 X-0.0683 Y-59.0243 F120.00
G01 X-0.1016 Y-59.0159 F120.00
G01 X-0.1339 Y-59.0044 F120.00
G01 X-0.1650 Y-58.9897 F120.00
G01 X-0.1944 Y-58.9720 F120.00
G01 X-0.2220 Y-58.9516 F120.00
G01 X-0.2475 Y-58.9285 F120.00
G01 X-0.2706 Y-58.9030 F120.00
G01 X-0.2910 Y-58.8754 F120.00
G01 X-0.3087 Y-58.8460 F120.00
G01 X-0.3234 Y-58.8149 F120.00
G01 X-0.3349 Y-58.7826 F120.00
G01 X-0.3433 Y-58.7493 F120.00
G01 X-0.3483 Y-58.7153 F120.00
G01 X-0.3500 Y-58.6810 F120.00
G01 X-0.3500 Y-0.0000 F120.00
G01 X-0.3483 Y0.0343 F120.00
G01 X-0.3433 Y0.0683 F120.00
G01 X-0.3349 Y0.1016 F120.00
G01 X-0.3234 Y0.1339 F120.00
G01 X-0.3087 Y0.1650 F120.00
G01 X-0.2910 Y0.1944 F120.00
G01 X-0.2706 Y0.2220 F120.00
G01 X-0.2475 Y0.2475 F120.00
G01 X-0.2220 Y0.2706 F120.00
G01 X-0.1944 Y0.2910 F120.00
G01 X-0.1650 Y0.3087 F120.00
G01 X-0.1339 Y0.3234 F120.00
G01 X-0.1016 Y0.3349 F120.00
G01 X-0.0683 Y0.3433 F120.00
G01 X-0.0343 Y0.3483 F120.00
G01 X0.0000 Y0.3500 F120.00
G01 X12.8710 Y0.3500 F120.00
G00 X12.8710 Y0.3500
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X0.0000 Y0.3500 F120.00
G01 X-0.0343 Y0.3483 F120.00
G01 X-0.0683 Y0.3433 F120.00
G01 X-0.1016 Y0.3349 F120.00
G01 X-0.1339 Y0.3234 F120.00
G01 X-0.1650 Y0.3087 F120.00
G01 X-0.1944 Y0.2910 F120.00
G01 X-0.2220 Y0.2706 F120.00
G01 X-0.2475 Y0.2475 F120.00
G01 X-0.2706 Y0.2220 F120.00
G01 X-0.2910 Y0.1944 F120.00
G01 X-0.3087 Y0.1650 F120.00
G01 X-0.3234 Y0.1339 F120.00
G01 X-0.3349 Y0.1016 F120.00
G01 X-0.3433 Y0.0683 F120.00
G01 X-0.3483 Y0.0343 F120.00
G01 X-0.3500 Y-0.0000 F120.00
G01 X-0.3500 Y-58.6810 F120.00
G01 X-0.3483 Y-58.7153 F120.00
G01 X-0.3433 Y-58.7493 F120.00
G01 X-0.3349 Y-58.7826 F120.00
G01 X-0.3234 Y-58.8149 F120.00
G01 X-0.3087 Y-58.8460 F120.00
G01 X-0.2910 Y-58.8754 F120.00
G01 X-0.2706 Y-58.9030 F120.00
G01 X-0.2475 Y-58.9285 F120.00
G01 X-0.2220 Y-58.9516 F120.00
G01 X-0.1944 Y-58.9720 F120.00
G01 X-0.1650 Y-58.9897 F120.00
G01 X-0.1339 Y-59.0044 F120.00
G01 X-0.1016 Y-59.0159 F120.00
G01 X-0.0683 Y-59.0243 F120.00
G01 X-0.0343 Y-59.0293 F120.00
G01 X0.0000 Y-59.0310 F120.00
G01 X12.8710 Y-59.0310 F120.00
G00 Z2.0000
G00 X13.4710 Y0.3500
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X26.1420 Y0.3500 F120.00
G01 X26.1763 Y0.3483 F120.00
G01 X26.2103 Y0.3433 F120.00
G01 X26.2436 Y0.3349 F120.00
G01 X26.2759 Y0.3234 F120.00
G01 X26.3070 Y0.3087 F120.00
G01 X26.3364 Y0.2910 F120.00
G01 X26.3640 Y0.2706 F120.00
G01 X26.3895 Y0.2475 F120.00
G01 X26.4126 Y0.2220 F120.00
G01 X26.4330 Y0.1944 F120.00
G01 X26.4507 Y0.1650 F120.00
G01 X26.4654 Y0.1339 F120.00
G01 X26.4769 Y0.1016 F120.00
G01 X26.4853 Y0.0683 F120.00
G01 X26.4903 Y0.0343 F120.00
G01 X26.4920 Y-0.0000 F120.00
G01 X26.4920 Y-58.6810 F120.00
G01 X26.4903 Y-58.7153 F120.00
G01 X26.4853 Y-58.7493 F120.00
G01 X26.4769 Y-58.7826 F120.00
G01 X26.4654 Y-58.8149 F120.00
G01 X26.4507 Y-58.8460 F120.00
G01 X26.4330 Y-58.8754 F120.00
G01 X26.4126 Y-58.9030 F120.00
G01 X26.3895 Y-58.9285 F120.00
G01 X26.3640 Y-58.9516 F120.00
G01 X26.3364 Y-58.9720 F120.00
G01 X26.3070 Y-58.9897 F120.00
G01 X26.2759 Y-59.0044 F120.00
G01 X26.2436 Y-59.0159 F120.00
G01 X26.2103 Y-59.0243 F120.00
G01 X26.1763 Y-59.0293 F120.00
G01 X26.1420 Y-59.0310 F120.00
G01 X13.4710 Y-59.0310 F120.00
G00 X13.4710 Y-59.0310
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X26.1420 Y-59.0310 F120.00
G01 X26.1763 Y-59.0293 F120.00
G01 X26.2103 Y-59.0243 F120.00
G01 X26.2436 Y-59.0159 F120.00
G01 X26.2759 Y-59.0044 F120.00
G01 X26.3070 Y-58.9897 F120.00
G01 X26.3364 Y-58.9720 F120.00
G01 X26.3640 Y-58.9516 F120.00
G01 X26.3895 Y-58.9285 F120.00
G01 X26.4126 Y-58.9030 F120.00
G01 X26.4330 Y-58.8754 F120.00
G01 X26.4507 Y-58.8460 F120.00
G01 X26.4654 Y-58.8149 F120.00
G01 X26.4769 Y-58.7826 F120.00
G01 X26.4853 Y-58.7493 F120.00
G01 X26.4903 Y-58.7153 F120.00
G01 X26.4920 Y-58.6810 F120.00
G01 X26.4920 Y-0.0000 F120.00
G01 X26.4903 Y0.0343 F120.00
G01 X26.4853 Y0.0683 F120.00
G01 X26.4769 Y0.1016 F120.00
G01 X26.4654 Y0.1339 F120.00
G01 X26.4507 Y0.1650 F120.00
G01 X26.4330 Y0.1944 F120.00
G01 X26.4126 Y0.2220 F120.00
G01 X26.3895 Y0.2475 F120.00
G01 X26.3640 Y0.2706 F120.00
G01 X26.3364 Y0.2910 F120.00
G01 X26.3070 Y0.3087 F120.00
G01 X26.2759 Y0.3234 F120.00
G01 X26.2436 Y0.3349 F120.00
G01 X26.2103 Y0.3433 F120.00
G01 X26.1763 Y0.3483 F120.00
G01 X26.1420 Y0.3500 F120.00
G01 X13.4710 Y0.3500 F120.00
G00 X13.4710 Y0.3500
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X26.1420 Y0.3500 F120.00
G01 X26.1763 Y0.3483 F120.00
G01 X26.2103 Y0.3433 F120.00
G01 X26.2436 Y0.3349 F120.00
G01 X26.2759 Y0.3234 F120.00
G01 X26.3070 Y0.3087 F120.00
G01 X26.3364 Y0.2910 F120.00
G01 X26.3640 Y0.2706 F120.00
G01 X26.3895 Y0.2475 F120.00
G01 X26.4126 Y0.2220 F120.00
G01 X26.4330 Y0.1944 F120.00
G01 X26.4507 Y0.1650 F120.00
G01 X26.4654 Y0.1339 F120.00
G01 X26.4769 Y0.1016 F120.00
G01 X26.4853 Y0.0683 F120.00
G01 X26.4903 Y0.0343 F120.00
G01 X26.4920 Y-0.0000 F120.00
G01 X26.4920 Y-58.6810 F120.00
G01 X26.4903 Y-58.7153 F120.00
G01 X26.4853 Y-58.7493 F120.00
G01 X26.4769 Y-58.7826 F120.00
G01 X26.4654 Y-58.8149 F120.00
G01 X26.4507 Y-58.8460 F120.00
G01 X26.4330 Y-58.8754 F120.00
G01 X26.4126 Y-58.9030 F120.00
G01 X26.3895 Y-58.9285 F120.00
G01 X26.3640 Y-58.9516 F120.00
G01 X26.3364 Y-58.9720 F120.00
G01 X26.3070 Y-58.9897 F120.00
G01 X26.2759 Y-59.0044 F120.00
G01 X26.2436 Y-59.0159 F120.00
G01 X26.2103 Y-59.0243 F120.00
G01 X26.1763 Y-59.0293 F120.00
G01 X26.1420 Y-59.0310 F120.00
G01 X13.4710 Y-59.0310 F120.00
G00 X13.4710 Y-59.0310
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X26.1420 Y-59.0310 F120.00
G01 X26.1763 Y-59.0293 F120.00
G01 X26.2103 Y-59.0243 F120.00
G01 X26.2436 Y-59.0159 F120.00
G01 X26.2759 Y-59.0044 F120.00
G01 X26.3070 Y-58.9897 F120.00
G01 X26.3364 Y-58.9720 F120.00
G01 X26.3640 Y-58.9516 F120.00
G01 X26.3895 Y-58.9285 F120.00
G01 X26.4126 Y-58.9030 F120.00
G01 X26.4330 Y-58.8754 F120.00
G01 X26.4507 Y-58.8460 F120.00
G01 X26.4654 Y-58.8149 F120.00
G01 X26.4769 Y-58.7826 F120.00
G01 X26.4853 Y-58.7493 F120.00
G01 X26.4903 Y-58.7153 F120.00
G01 X26.4920 Y-58.6810 F120.00
G01 X26.4920 Y-0.0000 F120.00
G01 X26.4903 Y0.0343 F120.00
G01 X26.4853 Y0.0683 F120.00
G01 X26.4769 Y0.1016 F120.00
G01 X26.4654 Y0.1339 F120.00
G01 X26.4507 Y0.1650 F120.00
G01 X26.4330 Y0.1944 F120.00
G01 X26.4126 Y0.2220 F120.00
G01 X26.3895 Y0.2475 F120.00
G01 X26.3640 Y0.2706 F120.00
G01 X26.3364 Y0.2910 F120.00
G01 X26.3070 Y0.3087 F120.00
G01 X26.2759 Y0.3234 F120.00
G01 X26.2436 Y0.3349 F120.00
G01 X26.2103 Y0.3433 F120.00
G01 X26.1763 Y0.3483 F120.00
G01 X26.1420 Y0.3500 F120.00
G01 X13.4710 Y0.3500 F120.00
G00 X13.4710 Y0.3500
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X26.1420 Y0.3500 F120.00
G01 X26.1763 Y0.3483 F120.00
G01 X26.2103 Y0.3433 F120.00
G01 X26.2436 Y0.3349 F120.00
G01 X26.2759 Y0.3234 F120.00
G01 X26.3070 Y0.3087 F120.00
G01 X26.3364 Y0.2910 F120.00
G01 X26.3640 Y0.2706 F120.00
G01 X26.3895 Y0.2475 F120.00
G01 X26.4126 Y0.2220 F120.00
G01 X26.4330 Y0.1944 F120.00
G01 X26.4507 Y0.1650 F120.00
G01 X26.4654 Y0.1339 F120.00
G01 X26.4769 Y0.1016 F120.00
G01 X26.4853 Y0.0683 F120.00
G01 X26.4903 Y0.0343 F120.00
G01 X26.4920 Y-0.0000 F120.00
G01 X26.4920 Y-58.6810 F120.00
G01 X26.4903 Y-58.7153 F120.00
G01 X26.4853 Y-58.7493 F120.00
G01 X26.4769 Y-58.7826 F120.00
G01 X26.4654 Y-58.8149 F120.00
G01 X26.4507 Y-58.8460 F120.00
G01 X26.4330 Y-58.8754 F120.00
G01 X26.4126 Y-58.9030 F120.00
G01 X26.3895 Y-58.9285 F120.00
G01 X26.3640 Y-58.9516 F120.00
G01 X26.3364 Y-58.9720 F120.00
G01 X26.3070 Y-58.9897 F120.00
G01 X26.2759 Y-59.0044 F120.00
G01 X26.2436 Y-59.0159 F120.00
G01 X26.2103 Y-59.0243 F120.00
G01 X26.1763 Y-59.0293 F120.00
G01 X26.1420 Y-59.0310 F120.00
G01 X13.4710 Y-59.0310 F120.00
G00 Z2.0000
G00 X39.2790 Y-59.0310
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X26.8580 Y-59.0310 F120.00
G01 X26.8237 Y-59.0293 F120.00
G01 X26.7897 Y-59.0243 F120.00
G01 X26.7564 Y-59.0159 F120.00
G01 X26.7241 Y-59.0044 F120.00
G01 X26.6930 Y-58.9897 F120.00
G01 X26.6636 Y-58.9720 F120.00
G01 X26.6360 Y-58.9516 F120.00
G01 X26.6105 Y-58.9285 F120.00
G01 X26.5874 Y-58.9030 F120.00
G01 X26.5670 Y-58.8754 F120.00
G01 X26.5493 Y-58.8460 F120.00
G01 X26.5346 Y-58.8149 F120.00
G01 X26.5231 Y-58.7826 F120.00
G01 X26.5147 Y-58.7493 F120.00
G01 X26.5097 Y-58.7153 F120.00
G01 X26.5080 Y-58.6810 F120.00
G01 X26.5080 Y-0.0000 F120.00
G01 X26.5097 Y0.0343 F120.00
G01 X26.5147 Y0.0683 F120.00
G01 X26.5231 Y0.1016 F120.00
G01 X26.5346 Y0.1339 F120.00
G01 X26.5493 Y0.1650 F120.00
G01 X26.5670 Y0.1944 F120.00
G01 X26.5874 Y0.2220 F120.00
G01 X26.6105 Y0.2475 F120.00
G01 X26.6360 Y0.2706 F120.00
G01 X26.6636 Y0.2910 F120.00
G01 X26.6930 Y0.3087 F120.00
G01 X26.7241 Y0.3234 F120.00
G01 X26.7564 Y0.3349 F120.00
G01 X26.7897 Y0.3433 F120.00
G01 X26.8237 Y0.3483 F120.00
G01 X26.8580 Y0.3500 F120.00
G01 X39.2790 Y0.3500 F120.00
G00 X39.2790 Y0.3500
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X26.8580 Y0.3500 F120.00
G01 X26.8237 Y0.3483 F120.00
G01 X26.7897 Y0.3433 F120.00
G01 X26.7564 Y0.3349 F120.00
G01 X26.7241 Y0.3234 F120.00
G01 X26.6930 Y0.3087 F120.00
G01 X26.6636 Y0.2910 F120.00
G01 X26.6360 Y0.2706 F120.00
G01 X26.6105 Y0.2475 F120.00
G01 X26.5874 Y0.2220 F120.00
G01 X26.5670 Y0.1944 F120.00
G01 X26.5493 Y0.1650 F120.00
G01 X26.5346 Y0.1339 F120.00
G01 X26.5231 Y0.1016 F120.00
G01 X26.5147 Y0.0683 F120.00
G01 X26.5097 Y0.0343 F120.00
G01 X26.5080 Y-0.0000 F120.00
G01 X26.5080 Y-58.6810 F120.00
G01 X26.5097 Y-58.7153 F120.00
G01 X26.5147 Y-58.7493 F120.00
G01 X26.5231 Y-58.7826 F120.00
G01 X26.5346 Y-58.8149 F120.00
G01 X26.5493 Y-58.8460 F120.00
G01 X26.5670 Y-58.8754 F120.00
G01 X26.5874 Y-58.9030 F120.00
G01 X26.6105 Y-58.9285 F120.00
G01 X26.6360 Y-58.9516 F120.00
G01 X26.6636 Y-58.9720 F120.00
G01 X26.6930 Y-58.9897 F120.00
G01 X26.7241 Y-59.0044 F120.00
G01 X26.7564 Y-59.0159 F120.00
G01 X26.7897 Y-59.0243 F120.00
G01 X26.8237 Y-59.0293 F120.00
G01 X26.8580 Y-59.0310 F120.00
G01 X39.2790 Y-59.0310 F120.00
G00 X39.2790 Y-59.0310
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X26.8580 Y-59.0310 F120.00
G01 X26.8237 Y-59.0293 F120.00
G01 X26.7897 Y-59.0243 F120.00
G01 X26.7564 Y-59.0159 F120.00
G01 X26.7241 Y-59.0044 F120.00
G01 X26.6930 Y-58.9897 F120.00
G01 X26.6636 Y-58.9720 F120.00
G01 X26.6360 Y-58.9516 F120.00
G01 X26.6105 Y-58.9285 F120.00
G01 X26.5874 Y-58.9030 F120.00
G01 X26.5670 Y-58.8754 F120.00
G01 X26.5493 Y-58.8460 F120.00
G01 X26.5346 Y-58.8149 F120.00
G01 X26.5231 Y-58.7826 F120.00
G01 X26.5147 Y-58.7493 F120.00
G01 X26.5097 Y-58.7153 F120.00
G01 X26.5080 Y-58.6810 F120.00
G01 X26.5080 Y-0.0000 F120.00
G01 X26.5097 Y0.0343 F120.00
G01 X26.5147 Y0.0683 F120.00
G01 X26.5231 Y0.1016 F120.00
G01 X26.5346 Y0.1339 F120.00
G01 X26.5493 Y0.1650 F120.00
G01 X26.5670 Y0.1944 F120.00
G01 X26.5874 Y0.2220 F120.00
G01 X26.6105 Y0.2475 F120.00
G01 X26.6360 Y0.2706 F120.00
G01 X26.6636 Y0.2910 F120.00
G01 X26.6930 Y0.3087 F120.00
G01 X26.7241 Y0.3234 F120.00
G01 X26.7564 Y0.3349 F120.00
G01 X26.7897 Y0.3433 F120.00
G01 X26.8237 Y0.3483 F120.00
G01 X26.8580 Y0.3500 F120.00
G01 X39.2790 Y0.3500 F120.00
G00 X39.2790 Y0.3500
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X26.8580 Y0.3500 F120.00
G01 X26.8237 Y0.3483 F120.00
G01 X26.7897 Y0.3433 F120.00
G01 X26.7564 Y0.3349 F120.00
G01 X26.7241 Y0.3234 F120.00
G01 X26.6930 Y0.3087 F120.00
G01 X26.6636 Y0.2910 F120.00
G01 X26.6360 Y0.2706 F120.00
G01 X26.6105 Y0.2475 F120.00
G01 X26.5874 Y0.2220 F120.00
G01 X26.5670 Y0.1944 F120.00
G01 X26.5493 Y0.1650 F120.00
G01 X26.5346 Y0.1339 F120.00
G01 X26.5231 Y0.1016 F120.00
G01 X26.5147 Y0.0683 F120.00
G01 X26.5097 Y0.0343 F120.00
G01 X26.5080 Y-0.0000 F120.00
G01 X26.5080 Y-58.6810 F120.00
G01 X26.5097 Y-58.7153 F120.00
G01 X26.5147 Y-58.7493 F120.00
G01 X26.5231 Y-58.7826 F120.00
G01 X26.5346 Y-58.8149 F120.00
G01 X26.5493 Y-58.8460 F120.00
G01 X26.5670 Y-58.8754 F120.00
G01 X26.5874 Y-58.9030 F120.00
G01 X26.6105 Y-58.9285 F120.00
G01 X26.6360 Y-58.9516 F120.00
G01 X26.6636 Y-58.9720 F120.00
G01 X26.6930 Y-58.9897 F120.00
G01 X26.7241 Y-59.0044 F120.00
G01 X26.7564 Y-59.0159 F120.00
G01 X26.7897 Y-59.0243 F120.00
G01 X26.8237 Y-59.0293 F120.00
G01 X26.8580 Y-59.0310 F120.00
G01 X39.2790 Y-59.0310 F120.00
G00 X39.2790 Y-59.0310
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X26.8580 Y-59.0310 F120.00
G01 X26.8237 Y-59.0293 F120.00
G01 X26.7897 Y-59.0243 F120.00
G01 X26.7564 Y-59.0159 F120.00
G01 X26.7241 Y-59.0044 F120.00
G01 X26.6930 Y-58.9897 F120.00
G01 X26.6636 Y-58.9720 F120.00
G01 X26.6360 Y-58.9516 F120.00
G01 X26.6105 Y-58.9285 F120.00
G01 X26.5874 Y-58.9030 F120.00
G01 X26.5670 Y-58.8754 F120.00
G01 X26.5493 Y-58.8460 F120.00
G01 X26.5346 Y-58.8149 F120.00
G01 X26.5231 Y-58.7826 F120.00
G01 X26.5147 Y-58.7493 F120.00
G01 X26.5097 Y-58.7153 F120.00
G01 X26.5080 Y-58.6810 F120.00
G01 X26.5080 Y-0.0000 F120.00
G01 X26.5097 Y0.0343 F120.00
G01 X26.5147 Y0.0683 F120.00
G01 X26.5231 Y0.1016 F120.00
G01 X26.5346 Y0.1339 F120.00
G01 X26.5493 Y0.1650 F120.00
G01 X26.5670 Y0.1944 F120.00
G01 X26.5874 Y0.2220 F120.00
G01 X26.6105 Y0.2475 F120.00
G01 X26.6360 Y0.2706 F120.00
G01 X26.6636 Y0.2910 F120.00
G01 X26.6930 Y0.3087 F120.00
G01 X26.7241 Y0.3234 F120.00
G01 X26.7564 Y0.3349 F120.00
G01 X26.7897 Y0.3433 F120.00
G01 X26.8237 Y0.3483 F120.00
G01 X26.8580 Y0.3500 F120.00
G01 X39.2790 Y0.3500 F120.00
G00 Z2.0000
G00 X40.7790 Y0.3500
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X53.0000 Y0.3500 F120.00
G01 X53.0343 Y0.3483 F120.00
G01 X53.0683 Y0.3433 F120.00
G01 X53.1016 Y0.3349 F120.00
G01 X53.1339 Y0.3234 F120.00
G01 X53.1650 Y0.3087 F120.00
G01 X53.1944 Y0.2910 F120.00
G01 X53.2220 Y0.2706 F120.00
G01 X53.2475 Y0.2475 F120.00
G01 X53.2706 Y0.2220 F120.00
G01 X53.2910 Y0.1944 F120.00
G01 X53.3087 Y0.1650 F120.00
G01 X53.3234 Y0.1339 F120.00
G01 X53.3349 Y0.1016 F120.00
G01 X53.3433 Y0.0683 F120.00
G01 X53.3483 Y0.0343 F120.00
G01 X53.3500 Y-0.0000 F120.00
G01 X53.3500 Y-58.6810 F120.00
G01 X53.3483 Y-58.7153 F120.00
G01 X53.3433 Y-58.7493 F120.00
G01 X53.3349 Y-58.7826 F120.00
G01 X53.3234 Y-58.8149 F120.00
G01 X53.3087 Y-58.8460 F120.00
G01 X53.2910 Y-58.8754 F120.00
G01 X53.2706 Y-58.9030 F120.00
G01 X53.2475 Y-58.9285 F120.00
G01 X53.2220 Y-58.9516 F120.00
G01 X53.1944 Y-58.9720 F120.00
G01 X53.1650 Y-58.9897 F120.00
G01 X53.1339 Y-59.0044 F120.00
G01 X53.1016 Y-59.0159 F120.00
G01 X53.0683 Y-59.0243 F120.00
G01 X53.0343 Y-59.0293 F120.00
G01 X53.0000 Y-59.0310 F120.00
G01 X40.7790 Y-59.0310 F120.00
G00 X40.7790 Y-59.0310
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X53.0000 Y-59.0310 F120.00
G01 X53.0343 Y-59.0293 F120.00
G01 X53.0683 Y-59.0243 F120.00
G01 X53.1016 Y-59.0159 F120.00
G01 X53.1339 Y-59.0044 F120.00
G01 X53.1650 Y-58.9897 F120.00
G01 X53.1944 Y-58.9720 F120.00
G01 X53.2220 Y-58.9516 F120.00
G01 X53.2475 Y-58.9285 F120.00
G01 X53.2706 Y-58.9030 F120.00
G01 X53.2910 Y-58.8754 F120.00
G01 X53.3087 Y-58.8460 F120.00
G01 X53.3234 Y-58.8149 F120.00
G01 X53.3349 Y-58.7826 F120.00
G01 X53.3433 Y-58.7493 F120.00
G01 X53.3483 Y-58.7153 F120.00
G01 X53.3500 Y-58.6810 F120.00
G01 X53.3500 Y-0.0000 F120.00
G01 X53.3483 Y0.0343 F120.00
G01 X53.3433 Y0.0683 F120.00
G01 X53.3349 Y0.1016 F120.00
G01 X53.3234 Y0.1339 F120.00
G01 X53.3087 Y0.1650 F120.00
G01 X53.2910 Y0.1944 F120.00
G01 X53.2706 Y0.2220 F120.00
G01 X53.2475 Y0.2475 F120.00
G01 X53.2220 Y0.2706 F120.00
G01 X53.1944 Y0.2910 F120.00
G01 X53.1650 Y0.3087 F120.00
G01 X53.1339 Y0.3234 F120.00
G01 X53.1016 Y0.3349 F120.00
G01 X53.0683 Y0.3433 F120.00
G01 X53.0343 Y0.3483 F120.00
G01 X53.0000 Y0.3500 F120.00
G01 X40.7790 Y0.3500 F120.00
G00 X40.7790 Y0.3500
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X53.0000 Y0.3500 F120.00
G01 X53.0343 Y0.3483 F120.00
G01 X53.0683 Y0.3433 F120.00
G01 X53.1016 Y0.3349 F120.00
G01 X53.1339 Y0.3234 F120.00
G01 X53.1650 Y0.3087 F120.00
G01 X53.1944 Y0.2910 F120.00
G01 X53.2220 Y0.2706 F120.00
G01 X53.2475 Y0.2475 F120.00
G01 X53.2706 Y0.2220 F120.00
G01 X53.2910 Y0.1944 F120.00
G01 X53.3087 Y0.1650 F120.00
G01 X53.3234 Y0.1339 F120.00
G01 X53.3349 Y0.1016 F120.00
G01 X53.3433 Y0.0683 F120.00
G01 X53.3483 Y0.0343 F120.00
G01 X53.3500 Y-0.0000 F120.00
G01 X53.3500 Y-58.6810 F120.00
G01 X53.3483 Y-58.7153 F120.00
G01 X53.3433 Y-58.7493 F120.00
G01 X53.3349 Y-58.7826 F120.00
G01 X53.3234 Y-58.8149 F120.00
G01 X53.3087 Y-58.8460 F120.00
G01 X53.2910 Y-58.8754 F120.00
G01 X53.2706 Y-58.9030 F120.00
G01 X53.2475 Y-58.9285 F120.00
G01 X53.2220 Y-58.9516 F120.00
G01 X53.1944 Y-58.9720 F120.00
G01 X53.1650 Y-58.9897 F120.00
G01 X53.1339 Y-59.0044 F120.00
G01 X53.1016 Y-59.0159 F120.00
G01 X53.0683 Y-59.0243 F120.00
G01 X53.0343 Y-59.0293 F120.00
G01 X53.0000 Y-59.0310 F120.00
G01 X40.7790 Y-59.0310 F120.00
G00 X40.7790 Y-59.0310
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X53.0000 Y-59.0310 F120.00
G01 X53.0343 Y-59.0293 F120.00
G01 X53.0683 Y-59.0243 F120.00
G01 X53.1016 Y-59.0159 F120.00
G01 X53.1339 Y-59.0044 F120.00
G01 X53.1650 Y-58.9897 F120.00
G01 X53.1944 Y-58.9720 F120.00
G01 X53.2220 Y-58.9516 F120.00
G01 X53.2475 Y-58.9285 F120.00
G01 X53.2706 Y-58.9030 F120.00
G01 X53.2910 Y-58.8754 F120.00
G01 X53.3087 Y-58.8460 F120.00
G01 X53.3234 Y-58.8149 F120.00
G01 X53.3349 Y-58.7826 F120.00
G01 X53.3433 Y-58.7493 F120.00
G01 X53.3483 Y-58.7153 F120.00
G01 X53.3500 Y-58.6810 F120.00
G01 X53.3500 Y-0.0000 F120.00
G01 X53.3483 Y0.0343 F120.00
G01 X53.3433 Y0.0683 F120.00
G01 X53.3349 Y0.1016 F120.00
G01 X53.3234 Y0.1339 F120.00
G01 X53.3087 Y0.1650 F120.00
G01 X53.2910 Y0.1944 F120.00
G01 X53.2706 Y0.2220 F120.00
G01 X53.2475 Y0.2475 F120.00
G01 X53.2220 Y0.2706 F120.00
G01 X53.1944 Y0.2910 F120.00
G01 X53.1650 Y0.3087 F120.00
G01 X53.1339 Y0.3234 F120.00
G01 X53.1016 Y0.3349 F120.00
G01 X53.0683 Y0.3433 F120.00
G01 X53.0343 Y0.3483 F120.00
G01 X53.0000 Y0.3500 F120.00
G01 X40.7790 Y0.3500 F120.00
G00 X40.7790 Y0.3500
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X53.0000 Y0.3500 F120.00
G01 X53.0343 Y0.3483 F120.00
G01 X53.0683 Y0.3433 F120.00
G01 X53.1016 Y0.3349 F120.00
G01 X53.1339 Y0.3234 F120.00
G01 X53.1650 Y0.3087 F120.00
G01 X53.1944 Y0.2910 F120.00
G01 X53.2220 Y0.2706 F120.00
G01 X53.2475 Y0.2475 F120.00
G01 X53.2706 Y0.2220 F120.00
G01 X53.2910 Y0.1944 F120.00
G01 X53.3087 Y0.1650 F120.00
G01 X53.3234 Y0.1339 F120.00
G01 X53.3349 Y0.1016 F120.00
G01 X53.3433 Y0.0683 F120.00
G01 X53.3483 Y0.0343 F120.00
G01 X53.3500 Y-0.0000 F120.00
G01 X53.3500 Y-58.6810 F120.00
G01 X53.3483 Y-58.7153 F120.00
G01 X53.3433 Y-58.7493 F120.00
G01 X53.3349 Y-58.7826 F120.00
G01 X53.3234 Y-58.8149 F120.00
G01 X53.3087 Y-58.8460 F120.00
G01 X53.2910 Y-58.8754 F120.00
G01 X53.2706 Y-58.9030 F120.00
G01 X53.2475 Y-58.9285 F120.00
G01 X53.2220 Y-58.9516 F120.00
G01 X53.1944 Y-58.9720 F120.00
G01 X53.1650 Y-58.9897 F120.00
G01 X53.1339 Y-59.0044 F120.00
G01 X53.1016 Y-59.0159 F120.00
G01 X53.0683 Y-59.0243 F120.00
G01 X53.0343 Y-59.0293 F120.00
G01 X53.0000 Y-59.0310 F120.00
G01 X40.7790 Y-59.0310 F120.00
G00 Z2.0000
G00 X66.2165 Y-59.0310
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X53.7955 Y-59.0310 F120.00
G01 X53.7612 Y-59.0293 F120.00
G01 X53.7272 Y-59.0243 F120.00
G01 X53.6939 Y-59.0159 F120.00
G01 X53.6616 Y-59.0044 F120.00
G01 X53.6305 Y-58.9897 F120.00
G01 X53.6011 Y-58.9720 F120.00
G01 X53.5735 Y-58.9516 F120.00
G01 X53.5480 Y-58.9285 F120.00
G01 X53.5249 Y-58.9030 F120.00
G01 X53.5045 Y-58.8754 F120.00
G01 X53.4868 Y-58.8460 F120.00
G01 X53.4721 Y-58.8149 F120.00
G01 X53.4606 Y-58.7826 F120.00
G01 X53.4522 Y-58.7493 F120.00
G01 X53.4472 Y-58.7153 F120.00
G01 X53.4455 Y-58.6810 F120.00
G01 X53.4455 Y-0.0000 F120.00
G01 X53.4472 Y0.0343 F120.00
G01 X53.4522 Y0.0683 F120.00
G01 X53.4606 Y0.1016 F120.00
G01 X53.4721 Y0.1339 F120.00
G01 X53.4868 Y0.1650 F120.00
G01 X53.5045 Y0.1944 F120.00
G01 X53.5249 Y0.2220 F120.00
G01 X53.5480 Y0.2475 F120.00
G01 X53.5735 Y0.2706 F120.00
G01 X53.6011 Y0.2910 F120.00
G01 X53.6305 Y0.3087 F120.00
G01 X53.6616 Y0.3234 F120.00
G01 X53.6939 Y0.3349 F120.00
G01 X53.7272 Y0.3433 F120.00
G01 X53.7612 Y0.3483 F120.00
G01 X53.7955 Y0.3500 F120.00
G01 X66.2165 Y0.3500 F120.00
G00 X66.2165 Y0.3500
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X53.7955 Y0.3500 F120.00
G01 X53.7612 Y0.3483 F120.00
G01 X53.7272 Y0.3433 F120.00
G01 X53.6939 Y0.3349 F120.00
G01 X53.6616 Y0.3234 F120.00
G01 X53.6305 Y0.3087 F120.00
G01 X53.6011 Y0.2910 F120.00
G01 X53.5735 Y0.2706 F120.00
G01 X53.5480 Y0.2475 F120.00
G01 X53.5249 Y0.2220 F120.00
G01 X53.5045 Y0.1944 F120.00
G01 X53.4868 Y0.1650 F120.00
G01 X53.4721 Y0.1339 F120.00
G01 X53.4606 Y0.1016 F120.00
G01 X53.4522 Y0.0683 F120.00
G01 X53.4472 Y0.0343 F120.00
G01 X53.4455 Y-0.0000 F120.00
G01 X53.4455 Y-58.6810 F120.00
G01 X53.4472 Y-58.7153 F120.00
G01 X53.4522 Y-58.7493 F120.00
G01 X53.4606 Y-58.7826 F120.00
G01 X53.4721 Y-58.8149 F120.00
G01 X53.4868 Y-58.8460 F120.00
G01 X53.5045 Y-58.8754 F120.00
G01 X53.5249 Y-58.9030 F120.00
G01 X53.5480 Y-58.9285 F120.00
G01 X53.5735 Y-58.9516 F120.00
G01 X53.6011 Y-58.9720 F120.00
G01 X53.6305 Y-58.9897 F120.00
G01 X53.6616 Y-59.0044 F120.00
G01 X53.6939 Y-59.0159 F120.00
G01 X53.7272 Y-59.0243 F120.00
G01 X53.7612 Y-59.0293 F120.00
G01 X53.7955 Y-59.0310 F120.00
G01 X66.2165 Y-59.0310 F120.00
G00 X66.2165 Y-59.0310
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X53.7955 Y-59.0310 F120.00
G01 X53.7612 Y-59.0293 F120.00
G01 X53.7272 Y-59.0243 F120.00
G01 X53.6939 Y-59.0159 F120.00
G01 X53.6616 Y-59.0044 F120.00
G01 X53.6305 Y-58.9897 F120.00
G01 X53.6011 Y-58.9720 F120.00
G01 X53.5735 Y-58.9516 F120.00
G01 X53.5480 Y-58.9285 F120.00
G01 X53.5249 Y-58.9030 F120.00
G01 X53.5045 Y-58.8754 F120.00
G01 X53.4868 Y-58.8460 F120.00
G01 X53.4721 Y-58.8149 F120.00
G01 X53.4606 Y-58.7826 F120.00
G01 X53.4522 Y-58.7493 F120.00
G01 X53.4472 Y-58.7153 F120.00
G01 X53.4455 Y-58.6810 F120.00
G01 X53.4455 Y-0.0000 F120.00
G01 X53.4472 Y0.0343 F120.00
G01 X53.4522 Y0.0683 F120.00
G01 X53.4606 Y0.1016 F120.00
G01 X53.4721 Y0.1339 F120.00
G01 X53.4868 Y0.1650 F120.00
G01 X53.5045 Y0.1944 F120.00
G01 X53.5249 Y0.2220 F120.00
G01 X53.5480 Y0.2475 F120.00
G01 X53.5735 Y0.2706 F120.00
G01 X53.6011 Y0.2910 F120.00
G01 X53.6305 Y0.3087 F120.00
G01 X53.6616 Y0.3234 F120.00
G01 X53.6939 Y0.3349 F120.00
G01 X53.7272 Y0.3433 F120.00
G01 X53.7612 Y0.3483 F120.00
G01 X53.7955 Y0.3500 F120.00
G01 X66.2165 Y0.3500 F120.00
G00 X66.2165 Y0.3500
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X53.7955 Y0.3500 F120.00
G01 X53.7612 Y0.3483 F120.00
G01 X53.7272 Y0.3433 F120.00
G01 X53.6939 Y0.3349 F120.00
G01 X53.6616 Y0.3234 F120.00
G01 X53.6305 Y0.3087 F120.00
G01 X53.6011 Y0.2910 F120.00
G01 X53.5735 Y0.2706 F120.00
G01 X53.5480 Y0.2475 F120.00
G01 X53.5249 Y0.2220 F120.00
G01 X53.5045 Y0.1944 F120.00
G01 X53.4868 Y0.1650 F120.00
G01 X53.4721 Y0.1339 F120.00
G01 X53.4606 Y0.1016 F120.00
G01 X53.4522 Y0.0683 F120.00
G01 X53.4472 Y0.0343 F120.00
G01 X53.4455 Y-0.0000 F120.00
G01 X53.4455 Y-58.6810 F120.00
G01 X53.4472 Y-58.7153 F120.00
G01 X53.4522 Y-58.7493 F120.00
G01 X53.4606 Y-58.7826 F120.00
G01 X53.4721 Y-58.8149 F120.00
G01 X53.4868 Y-58.8460 F120.00
G01 X53.5045 Y-58.8754 F120.00
G01 X53.5249 Y-58.9030 F120.00
G01 X53.5480 Y-58.9285 F120.00
G01 X53.5735 Y-58.9516 F120.00
G01 X53.6011 Y-58.9720 F120.00
G01 X53.6305 Y-58.9897 F120.00
G01 X53.6616 Y-59.0044 F120.00
G01 X53.6939 Y-59.0159 F120.00
G01 X53.7272 Y-59.0243 F120.00
G01 X53.7612 Y-59.0293 F120.00
G01 X53.7955 Y-59.0310 F120.00
G01 X66.2165 Y-59.0310 F120.00
G00 X66.2165 Y-59.0310
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X53.7955 Y-59.0310 F120.00
G01 X53.7612 Y-59.0293 F120.00
G01 X53.7272 Y-59.0243 F120.00
G01 X53.6939 Y-59.0159 F120.00
G01 X53.6616 Y-59.0044 F120.00
G01 X53.6305 Y-58.9897 F120.00
G01 X53.6011 Y-58.9720 F120.00
G01 X53.5735 Y-58.9516 F120.00
G01 X53.5480 Y-58.9285 F120.00
G01 X53.5249 Y-58.9030 F120.00
G01 X53.5045 Y-58.8754 F120.00
G01 X53.4868 Y-58.8460 F120.00
G01 X53.4721 Y-58.8149 F120.00
G01 X53.4606 Y-58.7826 F120.00
G01 X53.4522 Y-58.7493 F120.00
G01 X53.4472 Y-58.7153 F120.00
G01 X53.4455 Y-58.6810 F120.00
G01 X53.4455 Y-0.0000 F120.00
G01 X53.4472 Y0.0343 F120.00
G01 X53.4522 Y0.0683 F120.00
G01 X53.4606 Y0.1016 F120.00
G01 X53.4721 Y0.1339 F120.00
G01 X53.4868 Y0.1650 F120.00
G01 X53.5045 Y0.1944 F120.00
G01 X53.5249 Y0.2220 F120.00
G01 X53.5480 Y0.2475 F120.00
G01 X53.5735 Y0.2706 F120.00
G01 X53.6011 Y0.2910 F120.00
G01 X53.6305 Y0.3087 F120.00
G01 X53.6616 Y0.3234 F120.00
G01 X53.6939 Y0.3349 F120.00
G01 X53.7272 Y0.3433 F120.00
G01 X53.7612 Y0.3483 F120.00
G01 X53.7955 Y0.3500 F120.00
G01 X66.2165 Y0.3500 F120.00
G00 Z2.0000
G00 X67.7165 Y0.3500
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X79.9375 Y0.3500 F120.00
G01 X79.9718 Y0.3483 F120.00
G01 X80.0058 Y0.3433 F120.00
G01 X80.0391 Y0.3349 F120.00
G01 X80.0714 Y0.3234 F120.00
G01 X80.1025 Y0.3087 F120.00
G01 X80.1319 Y0.2910 F120.00
G01 X80.1595 Y0.2706 F120.00
G01 X80.1850 Y0.2475 F120.00
G01 X80.2081 Y0.2220 F120.00
G01 X80.2285 Y0.1944 F120.00
G01 X80.2462 Y0.1650 F120.00
G01 X80.2609 Y0.1339 F120.00
G01 X80.2724 Y0.1016 F120.00
G01 X80.2808 Y0.0683 F120.00
G01 X80.2858 Y0.0343 F120.00
G01 X80.2875 Y-0.0000 F120.00
G01 X80.2875 Y-58.6810 F120.00
G01 X80.2858 Y-58.7153 F120.00
G01 X80.2808 Y-58.7493 F120.00
G01 X80.2724 Y-58.7826 F120.00
G01 X80.2609 Y-58.8149 F120.00
G01 X80.2462 Y-58.8460 F120.00
G01 X80.2285 Y-58.8754 F120.00
G01 X80.2081 Y-58.9030 F120.00
G01 X80.1850 Y-58.9285 F120.00
G01 X80.1595 Y-58.9516 F120.00
G01 X80.1319 Y-58.9720 F120.00
G01 X80.1025 Y-58.9897 F120.00
G01 X80.0714 Y-59.0044 F120.00
G01 X80.0391 Y-59.0159 F120.00
G01 X80.0058 Y-59.0243 F120.00
G01 X79.9718 Y-59.0293 F120.00
G01 X79.9375 Y-59.0310 F120.00
G01 X67.7165 Y-59.0310 F120.00
G00 X67.7165 Y-59.0310
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X79.9375 Y-59.0310 F120.00
G01 X79.9718 Y-59.0293 F120.00
G01 X80.0058 Y-59.0243 F120.00
G01 X80.0391 Y-59.0159 F120.00
G01 X80.0714 Y-59.0044 F120.00
G01 X80.1025 Y-58.9897 F120.00
G01 X80.1319 Y-58.9720 F120.00
G01 X80.1595 Y-58.9516 F120.00
G01 X80.1850 Y-58.9285 F120.00
G01 X80.2081 Y-58.9030 F120.00
G01 X80.2285 Y-58.8754 F120.00
G01 X80.2462 Y-58.8460 F120.00
G01 X80.2609 Y-58.8149 F120.00
G01 X80.2724 Y-58.7826 F120.00
G01 X80.2808 Y-58.7493 F120.00
G01 X80.2858 Y-58.7153 F120.00
G01 X80.2875 Y-58.6810 F120.00
G01 X80.2875 Y-0.0000 F120.00
G01 X80.2858 Y0.0343 F120.00
G01 X80.2808 Y0.0683 F120.00
G01 X80.2724 Y0.1016 F120.00
G01 X80.2609 Y0.1339 F120.00
G01 X80.2462 Y0.1650 F120.00
G01 X80.2285 Y0.1944 F120.00
G01 X80.2081 Y0.2220 F120.00
G01 X80.1850 Y0.2475 F120.00
G01 X80.1595 Y0.2706 F120.00
G01 X80.1319 Y0.2910 F120.00
G01 X80.1025 Y0.3087 F120.00
G01 X80.0714 Y0.3234 F120.00
G01 X80.0391 Y0.3349 F120.00
G01 X80.0058 Y0.3433 F120.00
G01 X79.9718 Y0.3483 F120.00
G01 X79.9375 Y0.3500 F120.00
G01 X67.7165 Y0.3500 F120.00
G00 X67.7165 Y0.3500
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X79.9375 Y0.3500 F120.00
G01 X79.9718 Y0.3483 F120.00
G01 X80.0058 Y0.3433 F120.00
G01 X80.0391 Y0.3349 F120.00
G01 X80.0714 Y0.3234 F120.00
G01 X80.1025 Y0.3087 F120.00
G01 X80.1319 Y0.2910 F120.00
G01 X80.1595 Y0.2706 F120.00
G01 X80.1850 Y0.2475 F120.00
G01 X80.2081 Y0.2220 F120.00
G01 X80.2285 Y0.1944 F120.00
G01 X80.2462 Y0.1650 F120.00
G01 X80.2609 Y0.1339 F120.00
G01 X80.2724 Y0.1016 F120.00
G01 X80.2808 Y0.0683 F120.00
G01 X80.2858 Y0.0343 F120.00
G01 X80.2875 Y-0.0000 F120.00
G01 X80.2875 Y-58.6810 F120.00
G01 X80.2858 Y-58.7153 F120.00
G01 X80.2808 Y-58.7493 F120.00
G01 X80.2724 Y-58.7826 F120.00
G01 X80.2609 Y-58.8149 F120.00
G01 X80.2462 Y-58.8460 F120.00
G01 X80.2285 Y-58.8754 F120.00
G01 X80.2081 Y-58.9030 F120.00
G01 X80.1850 Y-58.9285 F120.00
G01 X80.1595 Y-58.9516 F120.00
G01 X80.1319 Y-58.9720 F120.00
G01 X80.1025 Y-58.9897 F120.00
G01 X80.0714 Y-59.0044 F120.00
G01 X80.0391 Y-59.0159 F120.00
G01 X80.0058 Y-59.0243 F120.00
G01 X79.9718 Y-59.0293 F120.00
G01 X79.9375 Y-59.0310 F120.00
G01 X67.7165 Y-59.0310 F120.00
G00 X67.7165 Y-59.0310
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X79.9375 Y-59.0310 F120.00
G01 X79.9718 Y-59.0293 F120.00
G01 X80.0058 Y-59.0243 F120.00
G01 X80.0391 Y-59.0159 F120.00
G01 X80.0714 Y-59.0044 F120.00
G01 X80.1025 Y-58.9897 F120.00
G01 X80.1319 Y-58.9720 F120.00
G01 X80.1595 Y-58.9516 F120.00
G01 X80.1850 Y-58.9285 F120.00
G01 X80.2081 Y-58.9030 F120.00
G01 X80.2285 Y-58.8754 F120.00
G01 X80.2462 Y-58.8460 F120.00
G01 X80.2609 Y-58.8149 F120.00
G01 X80.2724 Y-58.7826 F120.00
G01 X80.2808 Y-58.7493 F120.00
G01 X80.2858 Y-58.7153 F120.00
G01 X80.2875 Y-58.6810 F120.00
G01 X80.2875 Y-0.0000 F120.00
G01 X80.2858 Y0.0343 F120.00
G01 X80.2808 Y0.0683 F120.00
G01 X80.2724 Y0.1016 F120.00
G01 X80.2609 Y0.1339 F120.00
G01 X80.2462 Y0.1650 F120.00
G01 X80.2285 Y0.1944 F120.00
G01 X80.2081 Y0.2220 F120.00
G01 X80.1850 Y0.2475 F120.00
G01 X80.1595 Y0.2706 F120.00
G01 X80.1319 Y0.2910 F120.00
G01 X80.1025 Y0.3087 F120.00
G01 X80.0714 Y0.3234 F120.00
G01 X80.0391 Y0.3349 F120.00
G01 X80.0058 Y0.3433 F120.00
G01 X79.9718 Y0.3483 F120.00
G01 X79.9375 Y0.3500 F120.00
G01 X67.7165 Y0.3500 F120.00
G00 X67.7165 Y0.3500
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X79.9375 Y0.3500 F120.00
G01 X79.9718 Y0.3483 F120.00
G01 X80.0058 Y0.3433 F120.00
G01 X80.0391 Y0.3349 F120.00
G01 X80.0714 Y0.3234 F120.00
G01 X80.1025 Y0.3087 F120.00
G01 X80.1319 Y0.2910 F120.00
G01 X80.1595 Y0.2706 F120.00
G01 X80.1850 Y0.2475 F120.00
G01 X80.2081 Y0.2220 F120.00
G01 X80.2285 Y0.1944 F120.00
G01 X80.2462 Y0.1650 F120.00
G01 X80.2609 Y0.1339 F120.00
G01 X80.2724 Y0.1016 F120.00
G01 X80.2808 Y0.0683 F120.00
G01 X80.2858 Y0.0343 F120.00
G01 X80.2875 Y-0.0000 F120.00
G01 X80.2875 Y-58.6810 F120.00
G01 X80.2858 Y-58.7153 F120.00
G01 X80.2808 Y-58.7493 F120.00
G01 X80.2724 Y-58.7826 F120.00
G01 X80.2609 Y-58.8149 F120.00
G01 X80.2462 Y-58.8460 F120.00
G01 X80.2285 Y-58.8754 F120.00
G01 X80.2081 Y-58.9030 F120.00
G01 X80.1850 Y-58.9285 F120.00
G01 X80.1595 Y-58.9516 F120.00
G01 X80.1319 Y-58.9720 F120.00
G01 X80.1025 Y-58.9897 F120.00
G01 X80.0714 Y-59.0044 F120.00
G01 X80.0391 Y-59.0159 F120.00
G01 X80.0058 Y-59.0243 F120.00
G01 X79.9718 Y-59.0293 F120.00
G01 X79.9375 Y-59.0310 F120.00
G01 X67.7165 Y-59.0310 F120.00
G00 Z2.0000
G01 F60.00

M5             
G00 Z15.0000
G00 X0.0000 Y0.0000                
T2
(MSG, Change to Tool Dia = 0.5000)
M0
G00 Z15.0000
        
M03
G01 F60.00
G00 X12.8710 Y0.3500
G01 F60.00
G01 Z-0.5000
G01 F60.00
G01 X13.4710 Y0.3500 F60.00
G00 X13.4710 Y0.3500
G01 F60.00
G01 Z-1.0000
G01 F60.00
G01 X12.8710 Y0.3500 F60.00
G00 X12.8710 Y0.3500
G01 F60.00
G01 Z-1.5000
G01 F60.00
G01 X13.4710 Y0.3500 F60.00
G00 X13.4710 Y0.3500
G01 F60.00
G01 Z-2.0000
G01 F60.00
G01 X12.8710 Y0.3500 F60.00
G00 X12.8710 Y0.3500
G01 F60.00
G01 Z-2.5000
G01 F60.00
G01 X13.4710 Y0.3500 F60.00
G00 Z2.0000
G00 X13.4710 Y-59.0310
G01 F60.00
G01 Z-0.5000
G01 F60.00
G01 X12.8710 Y-59.0310 F60.00
G00 X12.8710 Y-59.0310
G01 F60.00
G01 Z-1.0000
G01 F60.00
G01 X13.4710 Y-59.0310 F60.00
G00 X13.4710 Y-59.0310
G01 F60.00
G01 Z-1.5000
G01 F60.00
G01 X12.8710 Y-59.0310 F60.00
G00 X12.8710 Y-59.0310
G01 F60.00
G01 Z-2.0000
G01 F60.00
G01 X13.4710 Y-59.0310 F60.00
G00 X13.4710 Y-59.0310
G01 F60.00
G01 Z-2.5000
G01 F60.00
G01 X12.8710 Y-59.0310 F60.00
G00 Z2.0000
M05
G00 Z2.0000
G00 Z15.00
G00 X0.0 Y0.0


