# Create a continuous plot for the PID metrics. use 'gnuplot -e 'PID_DATA=<filepath>' <script_name>

set terminal dumb size 120, 30; 
set autoscale; 

set title "GnuPlot - Fan Speed  and CPU Temperature"
set ylabel "Factor"
set y2tics
set xlabel "Time (step)

while (1) {
	replot 'PID_DATA' using 1:2 with lines title "P",
 	'PID_DATA' using 1:3 with lines title "I",
 	'PID_DATA' using 1:4 with lines title "D",
 	'PID_DATA' using 1:5 with lines title "RES"
}
