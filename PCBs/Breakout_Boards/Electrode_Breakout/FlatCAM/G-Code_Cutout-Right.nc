(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: Electrode_Breakout-Edge_Cuts.gbr_cutout_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Tuesday, 02 April 2024 at 22:10)

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

(X range:   60.6000 ...   91.6500  mm)
(Y range:  -66.1500 ...    0.4000  mm)

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
G00 X60.6000 Y-32.0250
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X60.6000 Y-0.0000 F120.00
G01 X60.6006 Y0.0221 F120.00
G01 X60.6040 Y0.0563 F120.00
G01 X60.6114 Y0.0948 F120.00
G01 X60.6226 Y0.1324 F120.00
G01 X60.6374 Y0.1688 F120.00
G01 X60.6582 Y0.2077 F120.00
G01 X60.6785 Y0.2380 F120.00
G01 X60.7020 Y0.2668 F120.00
G01 X60.7332 Y0.2980 F120.00
G01 X60.7619 Y0.3214 F120.00
G01 X60.7923 Y0.3418 F120.00
G01 X60.8312 Y0.3626 F120.00
G01 X60.8638 Y0.3761 F120.00
G01 X60.9004 Y0.3874 F120.00
G01 X60.9437 Y0.3960 F120.00
G01 X60.9779 Y0.3994 F120.00
G01 X61.0000 Y0.4000 F120.00
G01 X75.4750 Y0.4000 F120.00
G00 X75.4750 Y0.4000
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X61.0000 Y0.4000 F120.00
G01 X60.9779 Y0.3994 F120.00
G01 X60.9437 Y0.3960 F120.00
G01 X60.9004 Y0.3874 F120.00
G01 X60.8638 Y0.3761 F120.00
G01 X60.8312 Y0.3626 F120.00
G01 X60.7923 Y0.3418 F120.00
G01 X60.7619 Y0.3214 F120.00
G01 X60.7332 Y0.2980 F120.00
G01 X60.7020 Y0.2668 F120.00
G01 X60.6785 Y0.2380 F120.00
G01 X60.6582 Y0.2077 F120.00
G01 X60.6374 Y0.1688 F120.00
G01 X60.6226 Y0.1324 F120.00
G01 X60.6114 Y0.0948 F120.00
G01 X60.6040 Y0.0563 F120.00
G01 X60.6006 Y0.0221 F120.00
G01 X60.6000 Y-0.0000 F120.00
G01 X60.6000 Y-32.0250 F120.00
G00 X60.6000 Y-32.0250
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X60.6000 Y-0.0000 F120.00
G01 X60.6006 Y0.0221 F120.00
G01 X60.6040 Y0.0563 F120.00
G01 X60.6114 Y0.0948 F120.00
G01 X60.6226 Y0.1324 F120.00
G01 X60.6374 Y0.1688 F120.00
G01 X60.6582 Y0.2077 F120.00
G01 X60.6785 Y0.2380 F120.00
G01 X60.7020 Y0.2668 F120.00
G01 X60.7332 Y0.2980 F120.00
G01 X60.7619 Y0.3214 F120.00
G01 X60.7923 Y0.3418 F120.00
G01 X60.8312 Y0.3626 F120.00
G01 X60.8638 Y0.3761 F120.00
G01 X60.9004 Y0.3874 F120.00
G01 X60.9437 Y0.3960 F120.00
G01 X60.9779 Y0.3994 F120.00
G01 X61.0000 Y0.4000 F120.00
G01 X75.4750 Y0.4000 F120.00
G00 X75.4750 Y0.4000
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X61.0000 Y0.4000 F120.00
G01 X60.9779 Y0.3994 F120.00
G01 X60.9437 Y0.3960 F120.00
G01 X60.9004 Y0.3874 F120.00
G01 X60.8638 Y0.3761 F120.00
G01 X60.8312 Y0.3626 F120.00
G01 X60.7923 Y0.3418 F120.00
G01 X60.7619 Y0.3214 F120.00
G01 X60.7332 Y0.2980 F120.00
G01 X60.7020 Y0.2668 F120.00
G01 X60.6785 Y0.2380 F120.00
G01 X60.6582 Y0.2077 F120.00
G01 X60.6374 Y0.1688 F120.00
G01 X60.6226 Y0.1324 F120.00
G01 X60.6114 Y0.0948 F120.00
G01 X60.6040 Y0.0563 F120.00
G01 X60.6006 Y0.0221 F120.00
G01 X60.6000 Y-0.0000 F120.00
G01 X60.6000 Y-32.0250 F120.00
G00 X60.6000 Y-32.0250
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X60.6000 Y-0.0000 F120.00
G01 X60.6006 Y0.0221 F120.00
G01 X60.6040 Y0.0563 F120.00
G01 X60.6114 Y0.0948 F120.00
G01 X60.6226 Y0.1324 F120.00
G01 X60.6374 Y0.1688 F120.00
G01 X60.6582 Y0.2077 F120.00
G01 X60.6785 Y0.2380 F120.00
G01 X60.7020 Y0.2668 F120.00
G01 X60.7332 Y0.2980 F120.00
G01 X60.7619 Y0.3214 F120.00
G01 X60.7923 Y0.3418 F120.00
G01 X60.8312 Y0.3626 F120.00
G01 X60.8638 Y0.3761 F120.00
G01 X60.9004 Y0.3874 F120.00
G01 X60.9437 Y0.3960 F120.00
G01 X60.9779 Y0.3994 F120.00
G01 X61.0000 Y0.4000 F120.00
G01 X75.4750 Y0.4000 F120.00
G00 Z2.0000
G00 X76.9750 Y0.4000
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X91.2500 Y0.4000 F120.00
G01 X91.2721 Y0.3994 F120.00
G01 X91.3063 Y0.3960 F120.00
G01 X91.3496 Y0.3874 F120.00
G01 X91.3824 Y0.3774 F120.00
G01 X91.4188 Y0.3626 F120.00
G01 X91.4577 Y0.3418 F120.00
G01 X91.4863 Y0.3227 F120.00
G01 X91.5168 Y0.2980 F120.00
G01 X91.5480 Y0.2668 F120.00
G01 X91.5698 Y0.2402 F120.00
G01 X91.5918 Y0.2077 F120.00
G01 X91.6126 Y0.1688 F120.00
G01 X91.6274 Y0.1324 F120.00
G01 X91.6386 Y0.0948 F120.00
G01 X91.6460 Y0.0563 F120.00
G01 X91.6494 Y0.0221 F120.00
G01 X91.6500 Y-0.0000 F120.00
G01 X91.6500 Y-32.0250 F120.00
G00 X91.6500 Y-32.0250
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X91.6500 Y-0.0000 F120.00
G01 X91.6494 Y0.0221 F120.00
G01 X91.6460 Y0.0563 F120.00
G01 X91.6386 Y0.0948 F120.00
G01 X91.6274 Y0.1324 F120.00
G01 X91.6126 Y0.1688 F120.00
G01 X91.5918 Y0.2077 F120.00
G01 X91.5698 Y0.2402 F120.00
G01 X91.5480 Y0.2668 F120.00
G01 X91.5168 Y0.2980 F120.00
G01 X91.4863 Y0.3227 F120.00
G01 X91.4577 Y0.3418 F120.00
G01 X91.4188 Y0.3626 F120.00
G01 X91.3824 Y0.3774 F120.00
G01 X91.3496 Y0.3874 F120.00
G01 X91.3063 Y0.3960 F120.00
G01 X91.2721 Y0.3994 F120.00
G01 X91.2500 Y0.4000 F120.00
G01 X76.9750 Y0.4000 F120.00
G00 X76.9750 Y0.4000
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X91.2500 Y0.4000 F120.00
G01 X91.2721 Y0.3994 F120.00
G01 X91.3063 Y0.3960 F120.00
G01 X91.3496 Y0.3874 F120.00
G01 X91.3824 Y0.3774 F120.00
G01 X91.4188 Y0.3626 F120.00
G01 X91.4577 Y0.3418 F120.00
G01 X91.4863 Y0.3227 F120.00
G01 X91.5168 Y0.2980 F120.00
G01 X91.5480 Y0.2668 F120.00
G01 X91.5698 Y0.2402 F120.00
G01 X91.5918 Y0.2077 F120.00
G01 X91.6126 Y0.1688 F120.00
G01 X91.6274 Y0.1324 F120.00
G01 X91.6386 Y0.0948 F120.00
G01 X91.6460 Y0.0563 F120.00
G01 X91.6494 Y0.0221 F120.00
G01 X91.6500 Y-0.0000 F120.00
G01 X91.6500 Y-32.0250 F120.00
G00 X91.6500 Y-32.0250
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X91.6500 Y-0.0000 F120.00
G01 X91.6494 Y0.0221 F120.00
G01 X91.6460 Y0.0563 F120.00
G01 X91.6386 Y0.0948 F120.00
G01 X91.6274 Y0.1324 F120.00
G01 X91.6126 Y0.1688 F120.00
G01 X91.5918 Y0.2077 F120.00
G01 X91.5698 Y0.2402 F120.00
G01 X91.5480 Y0.2668 F120.00
G01 X91.5168 Y0.2980 F120.00
G01 X91.4863 Y0.3227 F120.00
G01 X91.4577 Y0.3418 F120.00
G01 X91.4188 Y0.3626 F120.00
G01 X91.3824 Y0.3774 F120.00
G01 X91.3496 Y0.3874 F120.00
G01 X91.3063 Y0.3960 F120.00
G01 X91.2721 Y0.3994 F120.00
G01 X91.2500 Y0.4000 F120.00
G01 X76.9750 Y0.4000 F120.00
G00 X76.9750 Y0.4000
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X91.2500 Y0.4000 F120.00
G01 X91.2721 Y0.3994 F120.00
G01 X91.3063 Y0.3960 F120.00
G01 X91.3496 Y0.3874 F120.00
G01 X91.3824 Y0.3774 F120.00
G01 X91.4188 Y0.3626 F120.00
G01 X91.4577 Y0.3418 F120.00
G01 X91.4863 Y0.3227 F120.00
G01 X91.5168 Y0.2980 F120.00
G01 X91.5480 Y0.2668 F120.00
G01 X91.5698 Y0.2402 F120.00
G01 X91.5918 Y0.2077 F120.00
G01 X91.6126 Y0.1688 F120.00
G01 X91.6274 Y0.1324 F120.00
G01 X91.6386 Y0.0948 F120.00
G01 X91.6460 Y0.0563 F120.00
G01 X91.6494 Y0.0221 F120.00
G01 X91.6500 Y-0.0000 F120.00
G01 X91.6500 Y-32.0250 F120.00
G00 Z2.0000
G00 X91.6500 Y-33.5250
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X91.6500 Y-65.7500 F120.00
G01 X91.6494 Y-65.7721 F120.00
G01 X91.6460 Y-65.8063 F120.00
G01 X91.6386 Y-65.8448 F120.00
G01 X91.6274 Y-65.8824 F120.00
G01 X91.6126 Y-65.9188 F120.00
G01 X91.5918 Y-65.9577 F120.00
G01 X91.5698 Y-65.9902 F120.00
G01 X91.5480 Y-66.0168 F120.00
G01 X91.5168 Y-66.0480 F120.00
G01 X91.4863 Y-66.0727 F120.00
G01 X91.4577 Y-66.0918 F120.00
G01 X91.4188 Y-66.1126 F120.00
G01 X91.3824 Y-66.1274 F120.00
G01 X91.3496 Y-66.1374 F120.00
G01 X91.3063 Y-66.1460 F120.00
G01 X91.2721 Y-66.1494 F120.00
G01 X91.2500 Y-66.1500 F120.00
G01 X76.9750 Y-66.1500 F120.00
G00 X76.9750 Y-66.1500
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X91.2500 Y-66.1500 F120.00
G01 X91.2721 Y-66.1494 F120.00
G01 X91.3063 Y-66.1460 F120.00
G01 X91.3496 Y-66.1374 F120.00
G01 X91.3824 Y-66.1274 F120.00
G01 X91.4188 Y-66.1126 F120.00
G01 X91.4577 Y-66.0918 F120.00
G01 X91.4863 Y-66.0727 F120.00
G01 X91.5168 Y-66.0480 F120.00
G01 X91.5480 Y-66.0168 F120.00
G01 X91.5698 Y-65.9902 F120.00
G01 X91.5918 Y-65.9577 F120.00
G01 X91.6126 Y-65.9188 F120.00
G01 X91.6274 Y-65.8824 F120.00
G01 X91.6386 Y-65.8448 F120.00
G01 X91.6460 Y-65.8063 F120.00
G01 X91.6494 Y-65.7721 F120.00
G01 X91.6500 Y-65.7500 F120.00
G01 X91.6500 Y-33.5250 F120.00
G00 X91.6500 Y-33.5250
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X91.6500 Y-65.7500 F120.00
G01 X91.6494 Y-65.7721 F120.00
G01 X91.6460 Y-65.8063 F120.00
G01 X91.6386 Y-65.8448 F120.00
G01 X91.6274 Y-65.8824 F120.00
G01 X91.6126 Y-65.9188 F120.00
G01 X91.5918 Y-65.9577 F120.00
G01 X91.5698 Y-65.9902 F120.00
G01 X91.5480 Y-66.0168 F120.00
G01 X91.5168 Y-66.0480 F120.00
G01 X91.4863 Y-66.0727 F120.00
G01 X91.4577 Y-66.0918 F120.00
G01 X91.4188 Y-66.1126 F120.00
G01 X91.3824 Y-66.1274 F120.00
G01 X91.3496 Y-66.1374 F120.00
G01 X91.3063 Y-66.1460 F120.00
G01 X91.2721 Y-66.1494 F120.00
G01 X91.2500 Y-66.1500 F120.00
G01 X76.9750 Y-66.1500 F120.00
G00 X76.9750 Y-66.1500
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X91.2500 Y-66.1500 F120.00
G01 X91.2721 Y-66.1494 F120.00
G01 X91.3063 Y-66.1460 F120.00
G01 X91.3496 Y-66.1374 F120.00
G01 X91.3824 Y-66.1274 F120.00
G01 X91.4188 Y-66.1126 F120.00
G01 X91.4577 Y-66.0918 F120.00
G01 X91.4863 Y-66.0727 F120.00
G01 X91.5168 Y-66.0480 F120.00
G01 X91.5480 Y-66.0168 F120.00
G01 X91.5698 Y-65.9902 F120.00
G01 X91.5918 Y-65.9577 F120.00
G01 X91.6126 Y-65.9188 F120.00
G01 X91.6274 Y-65.8824 F120.00
G01 X91.6386 Y-65.8448 F120.00
G01 X91.6460 Y-65.8063 F120.00
G01 X91.6494 Y-65.7721 F120.00
G01 X91.6500 Y-65.7500 F120.00
G01 X91.6500 Y-33.5250 F120.00
G00 X91.6500 Y-33.5250
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X91.6500 Y-65.7500 F120.00
G01 X91.6494 Y-65.7721 F120.00
G01 X91.6460 Y-65.8063 F120.00
G01 X91.6386 Y-65.8448 F120.00
G01 X91.6274 Y-65.8824 F120.00
G01 X91.6126 Y-65.9188 F120.00
G01 X91.5918 Y-65.9577 F120.00
G01 X91.5698 Y-65.9902 F120.00
G01 X91.5480 Y-66.0168 F120.00
G01 X91.5168 Y-66.0480 F120.00
G01 X91.4863 Y-66.0727 F120.00
G01 X91.4577 Y-66.0918 F120.00
G01 X91.4188 Y-66.1126 F120.00
G01 X91.3824 Y-66.1274 F120.00
G01 X91.3496 Y-66.1374 F120.00
G01 X91.3063 Y-66.1460 F120.00
G01 X91.2721 Y-66.1494 F120.00
G01 X91.2500 Y-66.1500 F120.00
G01 X76.9750 Y-66.1500 F120.00
G00 Z2.0000
G00 X75.4750 Y-66.1500
G01 F60.00
G01 Z-0.5000
G01 F120.00
G01 X61.0000 Y-66.1500 F120.00
G01 X60.9779 Y-66.1494 F120.00
G01 X60.9437 Y-66.1460 F120.00
G01 X60.9004 Y-66.1374 F120.00
G01 X60.8676 Y-66.1274 F120.00
G01 X60.8312 Y-66.1126 F120.00
G01 X60.7923 Y-66.0918 F120.00
G01 X60.7598 Y-66.0698 F120.00
G01 X60.7332 Y-66.0480 F120.00
G01 X60.7020 Y-66.0168 F120.00
G01 X60.6802 Y-65.9902 F120.00
G01 X60.6582 Y-65.9577 F120.00
G01 X60.6374 Y-65.9188 F120.00
G01 X60.6226 Y-65.8824 F120.00
G01 X60.6114 Y-65.8448 F120.00
G01 X60.6040 Y-65.8063 F120.00
G01 X60.6006 Y-65.7721 F120.00
G01 X60.6000 Y-65.7500 F120.00
G01 X60.6000 Y-33.5250 F120.00
G00 X60.6000 Y-33.5250
G01 F60.00
G01 Z-1.0000
G01 F120.00
G01 X60.6000 Y-65.7500 F120.00
G01 X60.6006 Y-65.7721 F120.00
G01 X60.6040 Y-65.8063 F120.00
G01 X60.6114 Y-65.8448 F120.00
G01 X60.6226 Y-65.8824 F120.00
G01 X60.6374 Y-65.9188 F120.00
G01 X60.6582 Y-65.9577 F120.00
G01 X60.6802 Y-65.9902 F120.00
G01 X60.7020 Y-66.0168 F120.00
G01 X60.7332 Y-66.0480 F120.00
G01 X60.7598 Y-66.0698 F120.00
G01 X60.7923 Y-66.0918 F120.00
G01 X60.8312 Y-66.1126 F120.00
G01 X60.8676 Y-66.1274 F120.00
G01 X60.9004 Y-66.1374 F120.00
G01 X60.9437 Y-66.1460 F120.00
G01 X60.9779 Y-66.1494 F120.00
G01 X61.0000 Y-66.1500 F120.00
G01 X75.4750 Y-66.1500 F120.00
G00 X75.4750 Y-66.1500
G01 F60.00
G01 Z-1.5000
G01 F120.00
G01 X61.0000 Y-66.1500 F120.00
G01 X60.9779 Y-66.1494 F120.00
G01 X60.9437 Y-66.1460 F120.00
G01 X60.9004 Y-66.1374 F120.00
G01 X60.8676 Y-66.1274 F120.00
G01 X60.8312 Y-66.1126 F120.00
G01 X60.7923 Y-66.0918 F120.00
G01 X60.7598 Y-66.0698 F120.00
G01 X60.7332 Y-66.0480 F120.00
G01 X60.7020 Y-66.0168 F120.00
G01 X60.6802 Y-65.9902 F120.00
G01 X60.6582 Y-65.9577 F120.00
G01 X60.6374 Y-65.9188 F120.00
G01 X60.6226 Y-65.8824 F120.00
G01 X60.6114 Y-65.8448 F120.00
G01 X60.6040 Y-65.8063 F120.00
G01 X60.6006 Y-65.7721 F120.00
G01 X60.6000 Y-65.7500 F120.00
G01 X60.6000 Y-33.5250 F120.00
G00 X60.6000 Y-33.5250
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X60.6000 Y-65.7500 F120.00
G01 X60.6006 Y-65.7721 F120.00
G01 X60.6040 Y-65.8063 F120.00
G01 X60.6114 Y-65.8448 F120.00
G01 X60.6226 Y-65.8824 F120.00
G01 X60.6374 Y-65.9188 F120.00
G01 X60.6582 Y-65.9577 F120.00
G01 X60.6802 Y-65.9902 F120.00
G01 X60.7020 Y-66.0168 F120.00
G01 X60.7332 Y-66.0480 F120.00
G01 X60.7598 Y-66.0698 F120.00
G01 X60.7923 Y-66.0918 F120.00
G01 X60.8312 Y-66.1126 F120.00
G01 X60.8676 Y-66.1274 F120.00
G01 X60.9004 Y-66.1374 F120.00
G01 X60.9437 Y-66.1460 F120.00
G01 X60.9779 Y-66.1494 F120.00
G01 X61.0000 Y-66.1500 F120.00
G01 X75.4750 Y-66.1500 F120.00
G00 X75.4750 Y-66.1500
G01 F60.00
G01 Z-2.5000
G01 F120.00
G01 X61.0000 Y-66.1500 F120.00
G01 X60.9779 Y-66.1494 F120.00
G01 X60.9437 Y-66.1460 F120.00
G01 X60.9004 Y-66.1374 F120.00
G01 X60.8676 Y-66.1274 F120.00
G01 X60.8312 Y-66.1126 F120.00
G01 X60.7923 Y-66.0918 F120.00
G01 X60.7598 Y-66.0698 F120.00
G01 X60.7332 Y-66.0480 F120.00
G01 X60.7020 Y-66.0168 F120.00
G01 X60.6802 Y-65.9902 F120.00
G01 X60.6582 Y-65.9577 F120.00
G01 X60.6374 Y-65.9188 F120.00
G01 X60.6226 Y-65.8824 F120.00
G01 X60.6114 Y-65.8448 F120.00
G01 X60.6040 Y-65.8063 F120.00
G01 X60.6006 Y-65.7721 F120.00
G01 X60.6000 Y-65.7500 F120.00
G01 X60.6000 Y-33.5250 F120.00
G00 Z2.0000
G01 F60.00

M5             
G00 Z15.0000
G00 X0.0000 Y0.0000                
T9999
(MSG, Change to Tool Dia = 0.5000)
M0
G00 Z15.0000
        
M03
G01 F60.00
G00 X60.6000 Y-32.0250
G01 F60.00
G01 Z-0.5000
G01 F60.00
G01 X60.6000 Y-33.5250 F60.00
G00 X60.6000 Y-33.5250
G01 F60.00
G01 Z-1.0000
G01 F60.00
G01 X60.6000 Y-32.0250 F60.00
G00 X60.6000 Y-32.0250
G01 F60.00
G01 Z-1.5000
G01 F60.00
G01 X60.6000 Y-33.5250 F60.00
G00 X60.6000 Y-33.5250
G01 F60.00
G01 Z-2.0000
G01 F60.00
G01 X60.6000 Y-32.0250 F60.00
G00 X60.6000 Y-32.0250
G01 F60.00
G01 Z-2.5000
G01 F60.00
G01 X60.6000 Y-33.5250 F60.00
G00 Z2.0000
G00 X91.6500 Y-32.0250
G01 F60.00
G01 Z-0.5000
G01 F60.00
G01 X91.6500 Y-33.5250 F60.00
G00 X91.6500 Y-33.5250
G01 F60.00
G01 Z-1.0000
G01 F60.00
G01 X91.6500 Y-32.0250 F60.00
G00 X91.6500 Y-32.0250
G01 F60.00
G01 Z-1.5000
G01 F60.00
G01 X91.6500 Y-33.5250 F60.00
G00 X91.6500 Y-33.5250
G01 F60.00
G01 Z-2.0000
G01 F60.00
G01 X91.6500 Y-32.0250 F60.00
G00 X91.6500 Y-32.0250
G01 F60.00
G01 Z-2.5000
G01 F60.00
G01 X91.6500 Y-33.5250 F60.00
G00 Z2.0000
G00 X76.9750 Y-66.1500
G01 F60.00
G01 Z-0.5000
G01 F60.00
G01 X75.4750 Y-66.1500 F60.00
G00 X75.4750 Y-66.1500
G01 F60.00
G01 Z-1.0000
G01 F60.00
G01 X76.9750 Y-66.1500 F60.00
G00 X76.9750 Y-66.1500
G01 F60.00
G01 Z-1.5000
G01 F60.00
G01 X75.4750 Y-66.1500 F60.00
G00 X75.4750 Y-66.1500
G01 F60.00
G01 Z-2.0000
G01 F60.00
G01 X76.9750 Y-66.1500 F60.00
G00 X76.9750 Y-66.1500
G01 F60.00
G01 Z-2.5000
G01 F60.00
G01 X75.4750 Y-66.1500 F60.00
G00 Z2.0000
G00 X75.4750 Y0.4000
G01 F60.00
G01 Z-0.5000
G01 F60.00
G01 X76.9750 Y0.4000 F60.00
G00 X76.9750 Y0.4000
G01 F60.00
G01 Z-1.0000
G01 F60.00
G01 X75.4750 Y0.4000 F60.00
G00 X75.4750 Y0.4000
G01 F60.00
G01 Z-1.5000
G01 F60.00
G01 X76.9750 Y0.4000 F60.00
G00 X76.9750 Y0.4000
G01 F60.00
G01 Z-2.0000
G01 F60.00
G01 X75.4750 Y0.4000 F60.00
G00 X75.4750 Y0.4000
G01 F60.00
G01 Z-2.5000
G01 F60.00
G01 X76.9750 Y0.4000 F60.00
G00 Z2.0000
M05
G00 Z2.0000
G00 Z15.00
G00 X0.0 Y0.0

