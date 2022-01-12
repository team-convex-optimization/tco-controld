set title "GnuPlot - Test"
set ylabel "Factor"
set y2tics
set xlabel "Time (step)"

x = 0
MAX_WINDOW = 300

TMP = real(system("tail -n 300 ../pid_data.txt"))

set yrange [-2:2]
set xrange [0:MAX_WINDOW]

plot TMP using 1:2 with lines title "P" , \
TMP using 1:3 with lines title "I", \
TMP using 1:4 with lines title "D", \
TMP using 1:5 with lines title "RES" 

while (1) {
replot
}



