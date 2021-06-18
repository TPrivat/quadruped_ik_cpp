# quadruped_ik_cpp
Inverse Kinematics for a Quadruped robot

This program is a translation of the python version found at https://spotmicroai.readthedocs.io/en/latest/kinematic/

REQUIREMENTS:
Eigen3

The main function uses gnuplot to plot body and leg positions to verify the library is working as expected. Positions are written to robot.dat

The code to produce the figure in gnuplot is below.

In GNUPlot
```
gnuplot> set xrange [-200:200]
gnuplot> set yrange [-200:200]
gnuplot> set zrange [-200:200]
gnuplot> set xlabel 'X'
gnuplot> set ylabel 'Y'
gnuplot> set zlabel 'Z'
gnuplot> set style line 1 linecolor rgb '#0060ad' linetype 1 linewidth 2
gnuplot> set style line 2 linecolor rgb '#dd181f' linetype 1 linewidth 2
gnuplot> set style line 3 linecolor rgb '#ffff00' linetype 1 linewidth 2
gnuplot> set style line 4 linecolor rgb '#00ff00' linetype 1 linewidth 2
gnuplot> set style line 5 linecolor rgb '#800080' linetype 1 linewidth 2
gnuplot> splot "robot.dat" index 0 with linespoints linestyle 1, \
'' index 1 with linespoints linestyle 2, \
'' index 2 with linespoints linestyle 3, \
'' index 3 with linespoints linestyle 4, \
'' index 4 with linespoints linestyle 5
```
